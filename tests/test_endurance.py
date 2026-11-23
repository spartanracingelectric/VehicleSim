import numpy as np
import pytest

from Vehicle.Battery.Configs.sr17 import create_sr17_battery
from Vehicle.Simulation.endurance import EnduranceRun, runEndurance

# two minutes of 40 kW pulls with a little regen between them, at 0.1 s steps
DEMAND_W = np.tile([40000.0] * 30 + [-5000.0] * 10, 30)


def test_run_matches_stepping_the_pack_by_hand_and_leaves_the_given_pack_alone():
    battery = create_sr17_battery()
    result = runEndurance(EnduranceRun(0.1, [30000.0] * 100, start_temp_C=35.0, battery=battery))
    by_hand = create_sr17_battery(initial_temp_C=35.0)
    by_hand.save_history = False
    for _ in range(100):
        by_hand.update(by_hand.currentForPower(30000.0, by_hand.getPackOpenCircuitVoltage_V()), 0.1)
    assert result.pack_temp_C[-1] == pytest.approx(by_hand.temp_C, abs=1e-9)
    assert (battery.temp_C, battery.soc) == (25.0, 1.0)


def test_overtemperature_shutdown_ends_the_run():
    result = runEndurance(EnduranceRun(0.1, [60000.0] * 3000, start_temp_C=58.0))
    assert result.shutdown and len(result.time_s) < 3000
    assert result.pack_temp_C[-1] >= 61.0


def test_a_pack_that_starts_too_hot_never_drives():
    result = runEndurance(EnduranceRun(0.1, [30000.0] * 10, start_temp_C=61.0))
    assert result.shutdown and not result.pack_current_A.any()


def test_limit_caps_discharge_and_lets_regen_through():
    free = runEndurance(EnduranceRun(0.1, DEMAND_W))
    limited = runEndurance(EnduranceRun(0.1, DEMAND_W, power_limit_W=20000.0))
    assert free.pack_power_W.max() == pytest.approx(40000.0, rel=0.01)
    assert limited.pack_power_W.max() == pytest.approx(20000.0, rel=0.01)
    assert limited.pack_power_W.min() < -4000.0
    assert limited.peak_temp_C < free.peak_temp_C
    assert limited.energy_used_kWh < free.energy_used_kWh


def test_limit_can_follow_the_bms_reading():
    def stopAt42(time_s, battery, bms):
        return 0.0 if bms.max_temperature_C >= 42 else 40000.0
    result = runEndurance(EnduranceRun(0.1, [40000.0] * 600, start_temp_C=40.0, power_limit_W=stopAt42))
    assert result.bms_max_temp_C.max() == 42
    assert result.pack_power_W[-1] == 0.0


def test_a_full_pack_takes_no_regen():
    result = runEndurance(EnduranceRun(0.1, [-5000.0] * 10))
    assert not result.pack_current_A.any() and result.soc[-1] == 1.0
