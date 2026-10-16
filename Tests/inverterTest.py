import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from Functions.clarkePark import clarke, inverseClarke, park, inversePark
from Vehicle.Inverter.InverterConfig import InverterConfig

# run: python Tests/inverterTest.py (or python -m Tests.inverterTest from the repo root)

BUS_VOLTAGE_V = 500.0
ANGLES_RAD = np.linspace(0, 2 * np.pi, 37)

def main():
    testPowerConversion()
    testClarkeRoundTrip()
    testParkRoundTrip()
    for modulation in ("spwm", "svm"):
        testAverageVoltageMatchesCommand(modulation)
        testVoltageSaturation(modulation)
    print("All inverter checks passed")

# Level 0: dc power and current for motoring and regen
def testPowerConversion():
    inverter = InverterConfig(con_current_A=300.0, max_dc_voltage_V=900.0, efficiency=0.97)

    inverter.updatePower(20000.0, BUS_VOLTAGE_V)
    assert np.isclose(inverter.getDcPower_W(), 20000.0 / 0.97)
    assert np.isclose(inverter.getDcCurrent_A(), 20000.0 / 0.97 / BUS_VOLTAGE_V)
    assert inverter.getLoss_W() > 0
    print(f"motoring: 20.0 kW ac -> {inverter.getDcPower_W() / 1000:.2f} kW dc, "
          f"{inverter.getDcCurrent_A():.2f} A, {inverter.getLoss_W():.0f} W loss")

    inverter.updatePower(-20000.0, BUS_VOLTAGE_V)
    assert np.isclose(inverter.getDcPower_W(), -20000.0 * 0.97)
    assert inverter.getDcCurrent_A() < 0
    assert inverter.getLoss_W() > 0
    print(f"regen:   -20.0 kW ac -> {inverter.getDcPower_W() / 1000:.2f} kW dc, "
          f"{inverter.getDcCurrent_A():.2f} A, {inverter.getLoss_W():.0f} W loss")

def testClarkeRoundTrip():
    rng = np.random.default_rng(0)
    for _ in range(100):
        a, b, c = rng.uniform(-400, 400, 3)
        assert np.allclose(inverseClarke(*clarke(a, b, c)), (a, b, c))
    print("clarke round trip ok")

def testParkRoundTrip():
    for theta_e_rad in ANGLES_RAD:
        d_syn, q_syn = park(120.0, -75.0, theta_e_rad)
        assert np.allclose(inversePark(d_syn, q_syn, theta_e_rad), (120.0, -75.0))
    print("park round trip ok")

# commands under the limit should come back out of the phase voltages unchanged
def testAverageVoltageMatchesCommand(modulation):
    inverter = InverterConfig(con_current_A=300.0, max_dc_voltage_V=900.0, modulation=modulation)
    max_voltage_V = inverter.getMaxVoltage_V(BUS_VOLTAGE_V)
    v_d_syn_V, v_q_syn_V = -0.3 * max_voltage_V, 0.9 * max_voltage_V

    for theta_e_rad in ANGLES_RAD:
        phase_voltages_V = inverter.updateVoltage(v_d_syn_V, v_q_syn_V, theta_e_rad, BUS_VOLTAGE_V)
        duty_cycles = inverter.getDutyCycles()
        assert not inverter.isVoltageSaturated()
        assert np.all((duty_cycles >= 0) & (duty_cycles <= 1))

        v_d_V, v_q_V, _ = clarke(*phase_voltages_V)
        assert np.allclose(park(v_d_V, v_q_V, theta_e_rad), (v_d_syn_V, v_q_syn_V))
    print(f"{modulation}: average voltage = command, max vector {max_voltage_V:.1f} V")

# commands over the limit get scaled onto the limit and flag saturation
def testVoltageSaturation(modulation):
    inverter = InverterConfig(con_current_A=300.0, max_dc_voltage_V=900.0, modulation=modulation)
    max_voltage_V = inverter.getMaxVoltage_V(BUS_VOLTAGE_V)

    for theta_e_rad in ANGLES_RAD:
        phase_voltages_V = inverter.updateVoltage(0.0, 2 * max_voltage_V, theta_e_rad, BUS_VOLTAGE_V)
        duty_cycles = inverter.getDutyCycles()
        assert inverter.isVoltageSaturated()
        assert np.all((duty_cycles >= -1e-12) & (duty_cycles <= 1 + 1e-12))

        v_d_V, v_q_V, _ = clarke(*phase_voltages_V)
        assert np.isclose(np.hypot(v_d_V, v_q_V), max_voltage_V)
    print(f"{modulation}: saturation ok")

if __name__ == "__main__":
    main()
