import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from copy import deepcopy
from Vehicle.Inverter.Configs.vesc_srinverter import vesc_srinverter
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5

# run: python Tests/motorDatasheetTest.py (or python -m Tests.motorDatasheetTest from the repo root)
# motor + the VESC inverter at fixed speeds on a fixed dc voltage, the current for full torque requested,
# against the curves on Vehicle/Motor/Configs/amk_motor_datasheet.pdf (those are with AMK's own inverter).
# the VESC holds a current, not a torque, and starts field weakening early (from 80 % of max duty), so the
# negative id's reluctance torque (Lq > Ld) puts it over AMK's curve at mid speeds. from 80 % of its max erpm
# (16000 rpm) it fades the current limit out, and once that's under the request it falls well under AMK's curve.
# checked: at least AMK's torque until the fade cuts the request, no more than it after, nothing at the speed limit

# read off the datasheet's torque-speed curves (Nm), knee = last speed with full 21 Nm
DATASHEET = {
    500: {13000: 21.0, 15000: 18.4, 17000: 16.3, 19000: 14.6, 19600: 14.1},
    600: {16000: 21.0, 17000: 19.8, 18000: 18.7, 19000: 17.7, 19600: 17.2},
}
TOLERANCE = 0.02
SETTLE_S = 0.3          # long enough for field weakening (it follows a slowly filtered duty) to settle
RAMP_FROM_RPM = 10000   # the dyno spins up to each speed over RAMP_S like the car would, instead of starting there:
RAMP_S = 0.2            # past ~18000 rpm at 500 V the back-emf is over what the bus can make, and from a cold start
                        # the currents run away before the duty-following field weakening has built up
AVERAGE_S = 0.02

def main():
    inverter = vesc_srinverter
    fade_start_rpm = inverter.erpm_start * inverter.max_erpm / (amk_dd5.Np / 2)
    max_rpm = inverter.max_erpm / (amk_dd5.Np / 2)
    # where the faded current limit drops under the request
    request_A = amk_dd5.find_iq_desired(amk_dd5.peakTorque_Nm)
    cut_rpm = fade_start_rpm + (1 - request_A / inverter.current_max_A) * (max_rpm - fade_start_rpm)
    for bus_voltage_V, points in DATASHEET.items():
        print(f"--- {bus_voltage_V} V ---")
        for speed_rpm, datasheet_Nm in points.items():
            torque_Nm = maxTorque_Nm(bus_voltage_V, speed_rpm)
            error = (torque_Nm - datasheet_Nm) / datasheet_Nm
            print(f"{speed_rpm:6d} rpm: datasheet {datasheet_Nm:5.1f} Nm, sim {torque_Nm:5.1f} Nm ({error:+.0%})")
            if speed_rpm <= cut_rpm:
                assert error >= -TOLERANCE, f"{speed_rpm} rpm at {bus_voltage_V} V makes less than AMK's curve"
            else:
                assert error <= TOLERANCE, f"{speed_rpm} rpm at {bus_voltage_V} V is past the fade but over AMK's curve"

    # the inverter fades the current out at its 100k erpm speed limit (20000 rpm with 5 pole pairs)
    at_limit_Nm = maxTorque_Nm(600, max_rpm)
    assert abs(at_limit_Nm) < 0.5, f"still {at_limit_Nm:.1f} Nm at the speed limit"
    print(f"at the {max_rpm:.0f} rpm speed limit: {at_limit_Nm:.2f} Nm")
    print(f"All datasheet checks passed (the fade cuts the {request_A:.1f} A request from {cut_rpm:.0f} rpm)")

# average torque at a speed and dc voltage with the current for full torque requested, once it's settled there
def maxTorque_Nm(bus_voltage_V, speed_rpm, steps_per_period=20):
    inverter = deepcopy(vesc_srinverter)
    motor = deepcopy(amk_dd5)

    control_period_s = inverter.control_period_s
    dt_s = control_period_s / steps_per_period
    num_periods = round(SETTLE_S / control_period_s)
    average_from = num_periods - round(AVERAGE_S / control_period_s)

    torques_Nm = []
    for period in range(num_periods):
        ramp = min(period * control_period_s / RAMP_S, 1.0)
        speed_radps = (RAMP_FROM_RPM + ramp * (speed_rpm - RAMP_FROM_RPM)) * 2 * np.pi / 60
        motor.set_wm(speed_radps)
        inverter.updateCurrentControl(motor.find_iq_desired(motor.peakTorque_Nm), motor, bus_voltage_V, control_period_s)
        for _ in range(steps_per_period):
            motor.update_from_phase_voltages(inverter.getPhaseVoltages_V(), dt_s, wm=speed_radps)
            inverter.measureCurrents(motor.id, motor.iq)
            if period >= average_from:
                torques_Nm.append(motor.Te)
    return float(np.mean(torques_Nm))

if __name__ == "__main__":
    main()
