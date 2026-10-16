import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from copy import deepcopy
from Vehicle.Inverter.Configs.inverter1 import inverter1
from Vehicle.Motor.Configs.amk_dd5 import amk_dd5

# run: python Tests/motorDatasheetTest.py (or python -m Tests.motorDatasheetTest from the repo root)
# motor + inverter at fixed speeds on a fixed dc voltage, full torque request, like the curves on
# Vehicle/Motor/Configs/amk_motor_datasheet.pdf, compared against those curves.
# the car's inverter uses field weakening, so those curves are the ones that get checked.
# up to the knee it should match closely, past it AMK's curves level off at a constant power
# (29 kW at 500 V, 35 kW at 600 V) that the sim's field weakening doesn't have, so it's looser there

# read off the datasheet's torque-speed curves (Nm), knee = last speed with full 21 Nm
DATASHEET = {
    500: {13000: 21.0, 15000: 18.4, 17000: 16.3, 19000: 14.6, 19600: 14.1},
    600: {16000: 21.0, 17000: 19.8, 18000: 18.7, 19000: 17.7, 19600: 17.2},
}
TOLERANCE_UP_TO_KNEE = 0.02
TOLERANCE_PAST_KNEE = 0.25
SETTLE_S = 0.15         # long enough for field weakening (Tn 6 ms) to settle
AVERAGE_S = 0.02

def main():
    worst_past_knee = 0.0
    for bus_voltage_V, points in DATASHEET.items():
        print(f"--- {bus_voltage_V} V, field weakening ---")
        for speed_rpm, datasheet_Nm in points.items():
            torque_Nm = maxTorque_Nm(bus_voltage_V, speed_rpm)
            error = (torque_Nm - datasheet_Nm) / datasheet_Nm
            past_knee = datasheet_Nm < amk_dd5.peakTorque_Nm
            tolerance = TOLERANCE_PAST_KNEE if past_knee else TOLERANCE_UP_TO_KNEE
            if past_knee:
                worst_past_knee = max(worst_past_knee, abs(error))
            print(f"{speed_rpm:6d} rpm: datasheet {datasheet_Nm:5.1f} Nm, sim {torque_Nm:5.1f} Nm ({error:+.0%})")
            assert abs(error) <= tolerance, f"{speed_rpm} rpm at {bus_voltage_V} V is {error:+.0%} off the datasheet"

    # the inverter fades the torque out at the motor's 20000 rpm speed limit
    at_limit_Nm = maxTorque_Nm(600, amk_dd5.maxSpeed_rpm)
    assert abs(at_limit_Nm) < 0.5, f"still {at_limit_Nm:.1f} Nm at the speed limit"
    print(f"at the {amk_dd5.maxSpeed_rpm:.0f} rpm speed limit: {at_limit_Nm:.2f} Nm")
    print(f"All datasheet checks passed (past the knee worst {worst_past_knee:.0%} off)")

# average torque at a fixed speed and dc voltage with full torque requested
def maxTorque_Nm(bus_voltage_V, speed_rpm, steps_per_period=20):
    inverter = deepcopy(inverter1)
    motor = deepcopy(amk_dd5)

    speed_radps = speed_rpm * 2 * np.pi / 60
    control_period_s = inverter.switching_period_s
    dt_s = control_period_s / steps_per_period
    num_periods = round(SETTLE_S / control_period_s)
    average_from = num_periods - round(AVERAGE_S / control_period_s)

    torques_Nm = []
    for period in range(num_periods):
        motor.set_wm(speed_radps)
        inverter.updateCurrentControl(motor.peakTorque_Nm, motor, bus_voltage_V, control_period_s)
        for _ in range(steps_per_period):
            motor.update_from_phase_voltages(inverter.getPhaseVoltages_V(), dt_s, wm=speed_radps)
            inverter.measureCurrents(motor.id, motor.iq)
            if period >= average_from:
                torques_Nm.append(motor.Te)
    return float(np.mean(torques_Nm))

if __name__ == "__main__":
    main()
