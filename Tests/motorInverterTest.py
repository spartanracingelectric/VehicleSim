import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from copy import deepcopy
from types import SimpleNamespace
from Vehicle.Inverter.Configs.inverter1 import inverter1
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5
from Vehicle.Battery.Configs.sr17 import sr17
from Data.plot import plotDrive

# run: python Tests/motorInverterTest.py (or python -m Tests.motorInverterTest from the repo root)
# motor on a dyno (speed held fixed) driven through the inverter, in the pdf timestep order:
# torque request -> iq* -> pi controller -> inverter voltage limit + phase voltages
# -> rotor-frame vd/vq -> motor currents, torque, power -> inverter dc current -> battery

DT_S = 5e-6    # motor electrical timestep, controller runs once per switching period

# torque steps at a fixed speed: motoring, more motoring, then regen
STEP_TEST = SimpleNamespace(
    name="torque steps at 6000 rpm",
    speed_rpm=6000.0,
    duration_s=0.06,
    torque_steps=[(0.0, 0.0), (0.005, 10.0), (0.025, 20.0), (0.045, -10.0)],
    initial_soc=0.9,    # leaves room for regen to charge it
)

# full torque at top speed on a part-empty pack: the lower pack voltage can't push 21 Nm worth of current
# against the motor's back-emf, so the inverter hits its voltage limit
SATURATION_TEST = SimpleNamespace(
    name="voltage limit at 17000 rpm, 30% pack",
    speed_rpm=17000.0,    # past where a 30% pack can make full torque (19000+ is past where it can even hold zero torque without field weakening)
    duration_s=0.2,    # field weakening (Tn 6 ms) needs time to build up from a cold start
    torque_steps=[(0.0, 0.0), (0.005, 21.0)],
    initial_soc=0.3,
)

def main():
    step_result = simulate(STEP_TEST)
    checkTorqueSteps(step_result)
    checkVoltageLimit(simulate(SATURATION_TEST))
    print("All motor + inverter checks passed")
    plotDrive(
        step_result.time_s,
        step_result.torque_request_Nm,
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

def getTorqueRequest(scenario, time_s):
    torque_Nm = 0.0
    for step_time_s, step_torque_Nm in scenario.torque_steps:
        if time_s >= step_time_s:
            torque_Nm = step_torque_Nm
    return torque_Nm

def simulate(scenario):
    inverter = deepcopy(inverter1)
    motor = deepcopy(amk_dd5)
    battery = deepcopy(sr17)
    battery.reset(initial_soc=scenario.initial_soc)

    control_period_s = inverter.switching_period_s
    steps_per_control_period = round(control_period_s / DT_S)
    speed_radps = scenario.speed_rpm * 2 * np.pi / 60
    time_s = np.arange(round(scenario.duration_s / DT_S)) * DT_S
    num_steps = len(time_s)

    result = SimpleNamespace(name=scenario.name, speed_radps=speed_radps, time_s=time_s,
                             steps_per_control_period=steps_per_control_period)
    for channel in ("torque_request_Nm", "torque_Nm", "iq_desired_A", "iq_A", "id_A", "vd_V", "vq_V",
                    "voltage_magnitude_V", "max_voltage_V", "mech_power_W", "motor_power_W",
                    "dc_power_W", "dc_current_A", "integral_eq", "copper_loss_W"):
        setattr(result, channel, np.zeros(num_steps))
    result.saturated = np.zeros(num_steps, dtype=bool)

    for step, t in enumerate(time_s):
        motor.set_wm(speed_radps)    # dyno holds the speed
        bus_voltage_V = battery.terminal_voltage_V

        # inverter's current controller runs once per switching period
        if step % steps_per_control_period == 0:
            torque_request_Nm = getTorqueRequest(scenario, t)
            max_voltage_V = inverter.getMaxVoltage_V(bus_voltage_V)
            inverter.updateCurrentControl(torque_request_Nm, motor, bus_voltage_V, control_period_s)

        # the phase voltages the inverter is holding go onto the motor, which works out what its rotor sees
        id_A, iq_A, Te, _, _, Pe = motor.update_from_phase_voltages(inverter.getPhaseVoltages_V(), DT_S, wm=speed_radps)
        inverter.measureCurrents(id_A, iq_A)
        vd_V, vq_V = motor.vd, motor.vq

        # motor electrical power -> dc side -> battery
        inverter.updatePower(Pe, bus_voltage_V)
        battery.update(inverter.getDcCurrent_A(), DT_S)

        result.torque_request_Nm[step] = torque_request_Nm
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

# last 2 ms before each torque step changes, when things have settled
def getSettledWindows(scenario, result):
    end_times_s = [step_time_s for step_time_s, _ in scenario.torque_steps[1:]] + [scenario.duration_s]
    return [(result.time_s > end_s - 0.002) & (result.time_s < end_s) for end_s in end_times_s]

def checkTorqueSteps(result):
    assert not result.saturated.any(), "should stay under the voltage limit"

    for window in getSettledWindows(STEP_TEST, result):
        request_Nm = result.torque_request_Nm[window][-1]
        torque_Nm = np.mean(result.torque_Nm[window])
        iq_error_A = np.mean(result.iq_A[window] - result.iq_desired_A[window])
        id_A = np.mean(result.id_A[window])

        # the controller only sees the current once per period, and in between it drifts as the rotor
        # turns (18 deg per period at 6000 rpm), so the average sits a bit off the sampled value
        assert abs(torque_Nm - request_Nm) < 0.02 * abs(request_Nm) + 0.05
        assert abs(id_A) < 2.0

        # power balance: electrical in = shaft power + copper loss
        motor_power_W = np.mean(result.motor_power_W[window])
        balance_W = np.mean(result.mech_power_W[window] + result.copper_loss_W[window])
        assert abs(motor_power_W - balance_W) < 0.01 * abs(motor_power_W) + 5

        dc_current_A = np.mean(result.dc_current_A[window])
        assert np.sign(dc_current_A) == np.sign(request_Nm) or request_Nm == 0
        print(f"{request_Nm:6.1f} Nm request -> {torque_Nm:6.2f} Nm, iq error {iq_error_A:+.2f} A, id {id_A:+.2f} A, "
              f"motor {motor_power_W / 1000:6.2f} kW = shaft + copper {balance_W / 1000:6.2f} kW, "
              f"dc {dc_current_A:+6.2f} A")

    max_id_A = np.max(np.abs(result.id_A))
    print(f"largest id kick during the steps: {max_id_A:.2f} A")

def checkVoltageLimit(result):
    window = getSettledWindows(SATURATION_TEST, result)[-1]
    request_Nm = result.torque_request_Nm[window][-1]
    torque_Nm = np.mean(result.torque_Nm[window])

    # the motor never sees more voltage than the inverter can make
    assert np.all(result.voltage_magnitude_V <= result.max_voltage_V * (1 + 1e-9))
    assert result.saturated[window].all()

    # torque falls short of the request but stays steady, and the integrator doesn't run away
    # (steady per control period, the rotor turns 60 deg per period here so there's ripple inside each one)
    assert torque_Nm < request_Nm
    torque_in_window_Nm = result.torque_Nm[window]
    whole_periods = len(torque_in_window_Nm) // result.steps_per_control_period * result.steps_per_control_period
    period_torque_Nm = torque_in_window_Nm[:whole_periods].reshape(-1, result.steps_per_control_period).mean(axis=1)
    assert np.std(period_torque_Nm) < 0.01 * torque_Nm
    assert np.ptp(result.integral_eq[window]) == 0

    # field weakening is pushing id negative to get torque out of the voltage that's left
    id_A = np.mean(result.id_A[window])
    assert id_A < -10, f"field weakening isn't engaged, id is {id_A:.1f} A"
    print(f"{request_Nm:.0f} Nm request at {SATURATION_TEST.speed_rpm:.0f} rpm -> {torque_Nm:.2f} Nm, "
          f"voltage {np.mean(result.voltage_magnitude_V[window]):.1f} V = limit {np.mean(result.max_voltage_V[window]):.1f} V, "
          f"field weakening id {id_A:.1f} A")

if __name__ == "__main__":
    main()
