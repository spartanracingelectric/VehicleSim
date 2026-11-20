import pytest

from Vehicle.Battery.Configs.sr17 import create_sr17_battery
from Vehicle.Simulation.endurance import EnduranceRun, runEndurance


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
