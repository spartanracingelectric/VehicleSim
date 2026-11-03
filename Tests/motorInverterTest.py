import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from copy import deepcopy
from types import SimpleNamespace
from Vehicle.Inverter.Configs.vesc_srinverter import vesc_srinverter
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5
from Vehicle.Battery.Configs.sr17 import sr17
from Data.plot import plotDrive

# run: python Tests/motorInverterTest.py (or python -m Tests.motorInverterTest from the repo root)
# motor on a dyno (speed held fixed) driven through the VESC inverter, in the pdf timestep order:
# current request -> limits -> pi controller -> inverter voltage limit + phase voltages
# -> rotor-frame vd/vq -> motor currents, torque, power -> inverter dc current -> battery

DT_S = 5e-6    # motor electrical timestep, stretched to fill each control period (the current loop runs at 26 kHz)

# current steps at a fixed speed: motoring, the firmware's full-pedal 76.3 A, then regen
STEP_TEST = SimpleNamespace(
    name="current steps at 6000 rpm",
    speed_rpm=6000.0,
    duration_s=0.08,
    current_steps=[(0.0, 0.0), (0.005, 40.0), (0.03, 76.3), (0.055, -30.0)],
    initial_soc=0.9,    # leaves room for regen to charge it
)

# the current for full torque at high speed on a part-empty pack: the lower pack voltage can't push it against
# the motor's back-emf, so the inverter hits its voltage limit and field weakening comes in
SATURATION_TEST = SimpleNamespace(
    name="voltage limit at 17000 rpm, 30% pack",
    speed_rpm=17000.0,
    duration_s=0.2,    # the filtered duty that field weakening follows takes a while to build up
    current_steps=[(0.0, 0.0), (0.005, amk_dd5.find_iq_desired(amk_dd5.peakTorque_Nm))],
    initial_soc=0.3,
)

def main():
    step_result = simulate(STEP_TEST)
    checkCurrentSteps(step_result)
    checkVoltageLimit(simulate(SATURATION_TEST))
    print("All motor + inverter checks passed")
    plotDrive(
        step_result.time_s,
        step_result.torque_command_Nm,
        step_result.torque_Nm,
        step_result.iq_desired_A,
        step_result.iq_A,
        step_result.id_A,
        step_result.vd_V,
        step_result.vq_V,
        step_result.voltage_magnitude_V,
        step_result.max_voltage_V,
        step_result.mech_power_W,
        step_result.motor_power_W,
        step_result.dc_power_W,
        step_result.dc_current_A,
        step_result.name,
    )

def getRequest_A(scenario, time_s):
    request_A = 0.0
    for step_time_s, step_request_A in scenario.current_steps:
        if time_s >= step_time_s:
            request_A = step_request_A
    return request_A

def simulate(scenario):
    inverter = deepcopy(vesc_srinverter)
    motor = deepcopy(amk_dd5)
    battery = deepcopy(sr17)
    battery.reset(initial_soc=scenario.initial_soc)

    control_period_s = inverter.control_period_s
    steps_per_control_period = round(control_period_s / DT_S)
    dt_s = control_period_s / steps_per_control_period
    speed_radps = scenario.speed_rpm * 2 * np.pi / 60
    time_s = np.arange(round(scenario.duration_s / dt_s)) * dt_s
    num_steps = len(time_s)

    result = SimpleNamespace(name=scenario.name, speed_radps=speed_radps, time_s=time_s,
                             steps_per_control_period=steps_per_control_period)
    for channel in ("torque_command_Nm", "torque_Nm", "iq_desired_A", "iq_A", "id_A", "vd_V", "vq_V",
                    "voltage_magnitude_V", "max_voltage_V", "mech_power_W", "motor_power_W",
                    "dc_power_W", "dc_current_A", "integral_eq", "copper_loss_W"):
        setattr(result, channel, np.zeros(num_steps))
    result.saturated = np.zeros(num_steps, dtype=bool)

    for step, t in enumerate(time_s):
        motor.set_wm(speed_radps)    # dyno holds the speed
        bus_voltage_V = battery.terminal_voltage_V

        # inverter's current controller runs once per control period
        if step % steps_per_control_period == 0:
            max_voltage_V = inverter.getMaxVoltage_V(bus_voltage_V)
            inverter.updateCurrentControl(getRequest_A(scenario, t), motor, bus_voltage_V, control_period_s)

        # the phase voltages the inverter is holding go onto the motor, which works out what its rotor sees
        id_A, iq_A, Te, _, _, Pe = motor.update_from_phase_voltages(inverter.getPhaseVoltages_V(), dt_s, wm=speed_radps)
        inverter.measureCurrents(id_A, iq_A)
        vd_V, vq_V = motor.vd, motor.vq

        # motor electrical power -> dc side -> battery
        inverter.updatePower(Pe, bus_voltage_V)
        battery.update(inverter.getDcCurrent_A(), dt_s)

        result.torque_command_Nm[step] = inverter.getTorqueCommand_Nm()
        result.torque_Nm[step] = Te
        result.iq_desired_A[step] = inverter.getCurrentDesired_A()[1]
        result.iq_A[step] = iq_A
        result.id_A[step] = id_A
        result.vd_V[step] = vd_V
        result.vq_V[step] = vq_V
        result.voltage_magnitude_V[step] = np.hypot(vd_V, vq_V)
        result.max_voltage_V[step] = max_voltage_V
        result.mech_power_W[step] = Te * speed_radps
        result.motor_power_W[step] = Pe
        result.copper_loss_W[step] = motor.P_copper
        result.dc_power_W[step] = inverter.getDcPower_W()
        result.dc_current_A[step] = inverter.getDcCurrent_A()
        result.integral_eq[step] = inverter.q_current_controller.getIntegral()
        result.saturated[step] = inverter.isVoltageSaturated()

    return result

# last 2 ms before each step changes, when things have settled
def getSettledWindows(scenario, result):
    end_times_s = [step_time_s for step_time_s, _ in scenario.current_steps[1:]] + [scenario.duration_s]
    return [(result.time_s > end_s - 0.002) & (result.time_s < end_s) for end_s in end_times_s]

# iq settles on the request, the torque is what that current makes (no field weakening at 6000 rpm, so iq times
# the motor's torque per amp), and the power balances
def checkCurrentSteps(result):
    assert not result.saturated.any(), "should stay under the voltage limit"
    Nm_per_A = 1 / amk_dd5.find_iq_desired(1.0)

    for window in getSettledWindows(STEP_TEST, result):
        command_A = result.iq_desired_A[window][-1]
        iq_A = np.mean(result.iq_A[window])
        torque_Nm = np.mean(result.torque_Nm[window])
        id_A = np.mean(result.id_A[window])

        assert abs(iq_A - command_A) < 0.02 * abs(command_A) + 0.5, f"iq {iq_A:.2f} A, commanded {command_A} A"
        assert abs(torque_Nm - Nm_per_A * command_A) < 0.02 * abs(Nm_per_A * command_A) + 0.05
        assert abs(id_A) < 2.0

        # power balance: electrical in = shaft power + copper loss
        motor_power_W = np.mean(result.motor_power_W[window])
        balance_W = np.mean(result.mech_power_W[window] + result.copper_loss_W[window])
        assert abs(motor_power_W - balance_W) < 0.01 * abs(motor_power_W) + 5

        dc_current_A = np.mean(result.dc_current_A[window])
        assert np.sign(dc_current_A) == np.sign(command_A) or command_A == 0
        print(f"{command_A:6.1f} A request -> iq {iq_A:6.2f} A, {torque_Nm:6.2f} Nm "
              f"(= {Nm_per_A:.4f} Nm/A x {command_A:.1f} A), id {id_A:+.2f} A, "
              f"motor {motor_power_W / 1000:6.2f} kW = shaft + copper {balance_W / 1000:6.2f} kW, dc {dc_current_A:+6.2f} A")

    max_id_A = np.max(np.abs(result.id_A))
    print(f"largest id kick during the steps: {max_id_A:.2f} A")

def checkVoltageLimit(result):
    window = getSettledWindows(SATURATION_TEST, result)[-1]
    request_A = SATURATION_TEST.current_steps[-1][1]
    torque_Nm = np.mean(result.torque_Nm[window])

    # the motor never sees more voltage than the inverter can make
    assert np.all(result.voltage_magnitude_V <= result.max_voltage_V * (1 + 1e-9))

    # torque falls short of what the request would make, but stays steady
    # (steady per control period, the rotor turns 25 deg per period here so there's ripple inside each one)
    assert torque_Nm < amk_dd5.peakTorque_Nm
    torque_in_window_Nm = result.torque_Nm[window]
    whole_periods = len(torque_in_window_Nm) // result.steps_per_control_period * result.steps_per_control_period
    period_torque_Nm = torque_in_window_Nm[:whole_periods].reshape(-1, result.steps_per_control_period).mean(axis=1)
    assert np.std(period_torque_Nm) < 0.01 * torque_Nm

    # field weakening is pushing id negative to get torque out of the voltage that's left
    id_A = np.mean(result.id_A[window])
    assert id_A < -10, f"field weakening isn't engaged, id is {id_A:.1f} A"
    print(f"{request_A:.1f} A request at {SATURATION_TEST.speed_rpm:.0f} rpm -> {torque_Nm:.2f} Nm, "
          f"voltage {np.mean(result.voltage_magnitude_V[window]):.1f} V, limit {np.mean(result.max_voltage_V[window]):.1f} V, "
          f"field weakening id {id_A:.1f} A")

if __name__ == "__main__":
    main()
