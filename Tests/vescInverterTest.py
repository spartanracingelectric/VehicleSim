import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from copy import deepcopy
from types import SimpleNamespace
from Vehicle.Inverter.Configs.vesc_srinverter import vesc_srinverter
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5
from Vehicle.Battery.Configs.sr17 import sr17
from Vehicle.MainConfigs.SR18 import SR18

# run: python Tests/vescInverterTest.py (or python -m Tests.vescInverterTest from the repo root)
# the VESC firmware's limits and settings on the AMK on a dyno: current and battery current limits,
# speed fade, field weakening, decoupling, and the car running at the 26 kHz current loop

DT_S = 5e-6    # stretched to fill the 38.5 us control period, like VehicleConfig does

# the firmware's default 60 A, low enough that the checks stay under the voltage limit at high speed
LOW_LIMIT = deepcopy(vesc_srinverter)
LOW_LIMIT.current_max_A = 60.0
LOW_LIMIT.current_min_A = -60.0

def main():
    testDecoupling()
    testCurrentLimit()
    testBatteryCurrentLimit()
    testSpeedFade()
    testFieldWeakening()
    testVehicleControlPeriod()
    print("All VESC inverter checks passed")

# motor on a dyno at speed_rpm, held at current_request_A for duration_s. returns the last 5 ms, averaged
def simulate(inverter, speed_rpm, current_request_A, duration_s=0.05, initial_soc=0.9):
    inverter = deepcopy(inverter)
    motor = deepcopy(amk_dd5)
    battery = deepcopy(sr17)
    battery.reset(initial_soc=initial_soc)

    control_period_s = inverter.control_period_s
    steps_per_control_period = round(control_period_s / DT_S)
    dt_s = control_period_s / steps_per_control_period
    speed_radps = speed_rpm * 2 * np.pi / 60
    num_periods = round(duration_s / control_period_s)
    settled_periods = round(0.005 / control_period_s)

    iq_A, id_A, torque_Nm, dc_current_A = [], [], [], []
    for period in range(num_periods):
        motor.set_wm(speed_radps)
        bus_voltage_V = battery.terminal_voltage_V
        inverter.updateCurrentControl(current_request_A, motor, bus_voltage_V, control_period_s)
        for _ in range(steps_per_control_period):
            _, _, Te, _, _, Pe = motor.update_from_phase_voltages(inverter.getPhaseVoltages_V(), dt_s, wm=speed_radps)
            inverter.measureCurrents(motor.id, motor.iq)
            inverter.updatePower(Pe, bus_voltage_V)
            battery.update(inverter.getDcCurrent_A(), dt_s)
            if period >= num_periods - settled_periods:
                iq_A.append(motor.iq)
                id_A.append(motor.id)
                torque_Nm.append(Te)
                dc_current_A.append(inverter.getDcCurrent_A())

    return SimpleNamespace(iq_A=np.mean(iq_A), id_A=np.mean(id_A), torque_Nm=np.mean(torque_Nm),
                           dc_current_A=np.mean(dc_current_A), saturated=inverter.isVoltageSaturated())

# 40 A from a standing start at 6000 rpm. with no decoupling (the firmware default) the integrator has to build
# the back-emf up itself and the cross terms (we * L = 0.75 ohm here) are 4x the 0.18 V/A Kp,
# so iq swings far past the request
def testDecoupling():
    off_inverter = deepcopy(vesc_srinverter)
    off_inverter.decoupling = "none"
    off = simulate(off_inverter, 6000.0, 40.0, duration_s=0.03)
    on = simulate(vesc_srinverter, 6000.0, 40.0, duration_s=0.03)
    assert abs(on.iq_A - 40.0) < 0.5
    print(f"40 A at 6000 rpm after 30 ms: decoupling off iq {off.iq_A:.1f} A id {off.id_A:.1f} A, "
          f"cross_bemf iq {on.iq_A:.1f} A id {on.id_A:.1f} A")

# asking for more than l_current_max gets l_current_max, both driving and braking
def testCurrentLimit():
    for request_A, expected_A in ((40.0, 40.0), (100.0, 60.0), (-100.0, -60.0)):
        result = simulate(LOW_LIMIT, 6000.0, request_A)
        assert not result.saturated
        assert abs(result.iq_A - expected_A) < 0.5, f"iq {result.iq_A:.2f} A, expected {expected_A} A"
        assert abs(result.id_A) < 2.0
        print(f"{request_A:6.1f} A request at 6000 rpm, 60 A limit -> iq {result.iq_A:6.2f} A, {result.torque_Nm:6.2f} Nm")

# battery current per inverter is capped (l_in_current_max): iq comes down until mod_q * iq fits
def testBatteryCurrentLimit():
    inverter = deepcopy(LOW_LIMIT)
    inverter.in_current_max_A = 20.0
    result = simulate(inverter, 15000.0, 60.0)
    assert result.iq_A < 55.0, "battery limit should have cut iq"
    assert abs(result.dc_current_A - 20.0) < 2.0, f"dc current {result.dc_current_A:.2f} A, limit 20 A"
    print(f"60 A request at 15000 rpm, 20 A battery limit -> iq {result.iq_A:.2f} A, dc {result.dc_current_A:.2f} A")

# l_erpm_start 0.8: at 90% of max speed the driving limit is halfway faded (60 -> 30 A). braking isn't faded,
# and a request already under the faded limit isn't scaled. (no field weakening here: its id would take its share
# of the total current limit)
def testSpeedFade():
    inverter = deepcopy(LOW_LIMIT)
    inverter.fw_current_max_A = 0.0
    driving = simulate(inverter, 18000.0, 60.0)
    braking = simulate(inverter, 18000.0, -60.0)
    small = simulate(inverter, 18000.0, 20.0)
    assert abs(driving.iq_A - 30.0) < 1.0, f"iq {driving.iq_A:.2f} A, expected 30 A"
    assert abs(braking.iq_A + 60.0) < 1.0, f"iq {braking.iq_A:.2f} A, expected -60 A"
    assert abs(small.iq_A - 20.0) < 1.0
    print(f"at 18000 rpm: 60 A -> {driving.iq_A:.2f} A, -60 A -> {braking.iq_A:.2f} A, 20 A -> {small.iq_A:.2f} A")

# with fw current set, high duty pushes id negative and gets more torque out of a low pack than with it off
def testFieldWeakening():
    no_fw = deepcopy(LOW_LIMIT)
    no_fw.fw_current_max_A = 0.0
    off = simulate(no_fw, 17000.0, 60.0, duration_s=0.1, initial_soc=0.3)
    on = simulate(LOW_LIMIT, 17000.0, 60.0, duration_s=0.1, initial_soc=0.3)
    # (with it off, id still drifts a little negative: the q axis is out of voltage and the d axis gets priority)
    assert on.id_A < off.id_A - 10.0, f"field weakening isn't engaged, id is {on.id_A:.1f} A"
    assert not on.saturated
    assert on.torque_Nm > off.torque_Nm
    print(f"60 A at 17000 rpm, 30% pack: fw off {off.torque_Nm:.2f} Nm (id {off.id_A:.1f} A), "
          f"fw on {on.torque_Nm:.2f} Nm (id {on.id_A:.1f} A)")

# the car runs one update per 26 kHz control loop, so a 10 ms VCU loop is 260 of them
def testVehicleControlPeriod():
    vehicle = deepcopy(SR18)
    assert np.isclose(vehicle.getControlPeriod_s(), 1 / 26000)
    vehicle.advance(0.01, current_request_A=40.0)
    assert vehicle.chassis.getSpeed_mps() > 0
    print(f"SR18: 10 ms at 40 A -> {vehicle.getTorqueFeedback_Nm():.2f} Nm per motor")

if __name__ == "__main__":
    main()
