import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from types import SimpleNamespace
from Vehicle.Inverter.InverterConfig import InverterConfig
from Data.plot import plotPwm

# run: python Tests/pwmTest.py (or python -m Tests.pwmTest from the repo root)
# Level 2 inverter driving a three-phase star r-l test load (made-up values, not the emrax).
# the same load is also driven by the Level 1 average voltages so the two can be compared.

BUS_VOLTAGE_V = 500.0
SWITCHING_FREQUENCY_HZ = 10000.0
ELECTRICAL_FREQUENCY_HZ = 100.0
VOLTAGE_COMMAND_V = 100.0            # size of the rotating voltage vector
LOAD_RESISTANCE_OHM = 0.5
LOAD_INDUCTANCE_H = 1e-3
STEPS_PER_SWITCHING_PERIOD = 200     # 0.5 us timestep at 10 kHz
NUM_ELECTRICAL_CYCLES = 3

def main():
    for modulation in ("spwm", "svm"):
        result = simulate(modulation)
        check(result)
    print("All pwm checks passed")

    # zoom in on three switching periods in the last electrical cycle
    zoom_start_s = (NUM_ELECTRICAL_CYCLES - 0.9) / ELECTRICAL_FREQUENCY_HZ
    plotPwm(
        result.time_s,
        result.carrier,
        result.duty_cycles,
        result.switched_voltages_V,
        result.average_voltages_V,
        result.phase_currents_A,
        result.average_currents_A,
        result.dc_current_A,
        result.average_dc_current_A,
        (zoom_start_s, zoom_start_s + 3 / SWITCHING_FREQUENCY_HZ),
    )

def simulate(modulation):
    inverter = InverterConfig(
        con_current_A=300.0,
        max_dc_voltage_V=900.0,
        modulation=modulation,
        switching_frequency_Hz=SWITCHING_FREQUENCY_HZ,
    )
    dt_s = inverter.switching_period_s / STEPS_PER_SWITCHING_PERIOD
    num_periods = round(NUM_ELECTRICAL_CYCLES * SWITCHING_FREQUENCY_HZ / ELECTRICAL_FREQUENCY_HZ)
    num_steps = num_periods * STEPS_PER_SWITCHING_PERIOD
    time_s = np.arange(num_steps) * dt_s

    result = SimpleNamespace(
        modulation=modulation,
        time_s=time_s,
        carrier=np.zeros(num_steps),
        duty_cycles=np.zeros((num_steps, 3)),
        switched_voltages_V=np.zeros((num_steps, 3)),
        average_voltages_V=np.zeros((num_steps, 3)),
        phase_currents_A=np.zeros((num_steps, 3)),
        average_currents_A=np.zeros((num_steps, 3)),
        dc_current_A=np.zeros(num_steps),
    )

    switched_currents_A = np.zeros(3)
    average_currents_A = np.zeros(3)

    for step, t in enumerate(time_s):
        # duty cycles get updated once per switching period, like the real controller
        if step % STEPS_PER_SWITCHING_PERIOD == 0:
            theta_e_rad = 2 * np.pi * ELECTRICAL_FREQUENCY_HZ * t
            inverter.updateVoltage(0.0, VOLTAGE_COMMAND_V, theta_e_rad, BUS_VOLTAGE_V)

        inverter.updateSwitching(t, switched_currents_A)

        upper, lower = inverter.getGateCommands()
        assert np.all(upper + lower == 1), "upper and lower igbt on together"

        result.carrier[step] = inverter.getCarrier(t)
        result.duty_cycles[step] = inverter.getDutyCycles()
        result.switched_voltages_V[step] = inverter.getSwitchedVoltages_V()
        result.average_voltages_V[step] = inverter.getPhaseVoltages_V()
        result.phase_currents_A[step] = switched_currents_A
        result.average_currents_A[step] = average_currents_A
        result.dc_current_A[step] = inverter.getDcCurrent_A()

        switched_currents_A = stepLoad(switched_currents_A, inverter.getSwitchedVoltages_V(), dt_s)
        average_currents_A = stepLoad(average_currents_A, inverter.getPhaseVoltages_V(), dt_s)

    # battery current averaged over each switching period
    period_dc_current_A = result.dc_current_A.reshape(num_periods, STEPS_PER_SWITCHING_PERIOD).mean(axis=1)
    result.average_dc_current_A = np.repeat(period_dc_current_A, STEPS_PER_SWITCHING_PERIOD)

    return result

# star-connected r-l load, exact step for a voltage held over dt
def stepLoad(currents_A, leg_voltages_V, dt_s):
    # the floating star point sits at the mean leg voltage, so the common part drops out
    load_voltages_V = leg_voltages_V - np.mean(leg_voltages_V)
    steady_currents_A = load_voltages_V / LOAD_RESISTANCE_OHM
    decay = np.exp(-LOAD_RESISTANCE_OHM * dt_s / LOAD_INDUCTANCE_H)
    return steady_currents_A + (currents_A - steady_currents_A) * decay

def check(result):
    steps_per_cycle = round(len(result.time_s) / NUM_ELECTRICAL_CYCLES)
    last_cycle = slice(-steps_per_cycle, None)

    # switched leg voltage averages out to the Level 1 voltage every switching period
    switched_V = result.switched_voltages_V.reshape(-1, STEPS_PER_SWITCHING_PERIOD, 3).mean(axis=1)
    average_V = result.average_voltages_V[::STEPS_PER_SWITCHING_PERIOD]
    voltage_error_V = np.max(np.abs(switched_V - average_V))
    assert voltage_error_V <= BUS_VOLTAGE_V / STEPS_PER_SWITCHING_PERIOD + 1e-9

    # switched currents follow the average-model currents, plus ripple
    peak_current_A = np.max(np.abs(result.average_currents_A[last_cycle]))
    ripple_A = np.max(np.abs(result.phase_currents_A - result.average_currents_A)[last_cycle])
    assert ripple_A < 0.1 * peak_current_A

    # ideal switches: power out of the battery = power burned in the load resistors
    battery_power_W = BUS_VOLTAGE_V * np.mean(result.dc_current_A[last_cycle])
    load_power_W = np.mean(LOAD_RESISTANCE_OHM * np.sum(result.phase_currents_A[last_cycle] ** 2, axis=1))
    assert np.isclose(battery_power_W, load_power_W, rtol=0.02)

    print(f"{result.modulation}: leg voltage avg error {voltage_error_V:.2f} V, "
          f"phase current peak {peak_current_A:.1f} A with ripple +/-{ripple_A:.1f} A, "
          f"dc current avg {np.mean(result.dc_current_A[last_cycle]):.2f} A, "
          f"battery {battery_power_W / 1000:.2f} kW vs load {load_power_W / 1000:.2f} kW")

if __name__ == "__main__":
    main()
