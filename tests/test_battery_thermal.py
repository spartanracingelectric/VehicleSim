import numpy as np
import pytest

from Vehicle import create_sr18
from Vehicle.Battery.Configs.sr17 import SR17_THERMAL_FITS, create_sr17_battery, create_sr17_thermal_model

# (sensor C, pack C) from LPTN-Cooling-Project scripts/models/thermal_models.py after 60 s at 120 A, then 20 s at -40 A,
# then 60 s parked. 1 s steps, start 30 C, sink 29.75 C
DERATING_REPO_C = [(64.346010, 37.901943), (53.175619, 40.521077), (43.206855, 42.323616)]


def test_pack_matches_the_derating_repo():
    battery = create_sr17_battery(initial_temp_C=30.0)
    battery.save_history = False
    for (sensor_C, pack_C), (current_A, seconds) in zip(DERATING_REPO_C, [(120.0, 60), (-40.0, 20), (0.0, 60)]):
        for _ in range(seconds):
            battery.update(current_A, 1.0)
        assert battery.temp_C == pytest.approx(sensor_C, abs=1e-5)
        assert battery.pack_node_temp_C == pytest.approx(pack_C, abs=1e-5)


def test_cooling_rates_are_real_numbers():
    model = create_sr17_thermal_model()
    assert not np.iscomplexobj(model.cooling_rates) and not np.iscomplexobj(model.temperature_modes)


def test_result_does_not_depend_on_the_time_step():
    one_step, many_steps = create_sr17_thermal_model(), create_sr17_thermal_model()
    one_step.update(4000.0, 60.0, 24.75)
    for _ in range(6000):
        many_steps.update(4000.0, 0.01, 24.75)
    assert many_steps.sensor_temp_C == pytest.approx(one_step.sensor_temp_C, abs=1e-9)
    assert many_steps.pack_temp_C == pytest.approx(one_step.pack_temp_C, abs=1e-9)


def test_trained_sink_follows_the_start_temperature_not_ambient():
    battery = create_sr17_battery(initial_temp_C=40.0, ambient_temp_C=25.0)
    assert battery.sink_temp_C == 39.75
    battery.reset(initial_temp_C=45.0)    # the call the SRE-VCU SIL harness makes before a run
    assert battery.sink_temp_C == 44.75


def test_ambient_sink_rule_uses_ambient_and_other_rules_are_rejected():
    assert create_sr17_battery(initial_temp_C=40.0, ambient_temp_C=25.0, sink_rule="ambient").sink_temp_C == 25.0
    with pytest.raises(ValueError, match="sink_rule"):
        create_sr17_battery(sink_rule="coolant")


def test_a_battery_created_hot_resets_hot():
    battery = create_sr17_battery(initial_soc=0.6, initial_temp_C=38.0)
    battery.update(50.0, 1.0)
    battery.reset()
    assert (battery.soc, battery.temp_C, battery.pack_node_temp_C) == (0.6, 38.0, 38.0)


def test_a_custom_battery_keeps_its_state_unless_the_car_is_told_otherwise():
    battery = create_sr17_battery(initial_soc=0.5, initial_temp_C=40.0, ambient_temp_C=35.0)
    kept = create_sr18(battery=battery).battery
    assert (kept.soc, kept.temp_C, kept.ambient_temp_C) == (0.5, 40.0, 35.0)
    changed = create_sr18(battery=battery, initial_temp_C=30.0).battery
    assert (changed.soc, changed.temp_C, changed.ambient_temp_C) == (0.5, 30.0, 35.0)


def test_car_uses_the_frozen_fit_unless_another_is_chosen():
    assert create_sr18().battery.thermal_model.r_th_c_per_w == SR17_THERMAL_FITS["frozen_2026_10_22"]["r_th_c_per_w"]
    refit = create_sr18(battery_thermal_fit="refit_2026_10_28").battery
    assert refit.thermal_model.r_th_c_per_w == SR17_THERMAL_FITS["refit_2026_10_28"]["r_th_c_per_w"]
    with pytest.raises(ValueError, match="thermal fit"):
        create_sr18(battery_thermal_fit="made_up")


# (sensor C, pack C) for the refit on the same drive as DERATING_REPO_C
def test_refit_matches_the_derating_repo():
    battery = create_sr17_battery(initial_temp_C=30.0, thermal_fit="refit_2026_10_28")
    battery.save_history = False
    expected_C = [(64.475973, 36.319329), (52.498412, 38.486183), (41.195703, 40.116120)]
    for (sensor_C, pack_C), (current_A, seconds) in zip(expected_C, [(120.0, 60), (-40.0, 20), (0.0, 60)]):
        for _ in range(seconds):
            battery.update(current_A, 1.0)
        assert battery.temp_C == pytest.approx(sensor_C, abs=1e-5)
        assert battery.pack_node_temp_C == pytest.approx(pack_C, abs=1e-5)


# whole-degree thermistor readings and a strict "above 60" check: the simulated bms trips at 61.0 C, not 60.0 C
@pytest.mark.parametrize("pack_temp_C, trips", [(60.9, False), (61.0, True)])
def test_bms_overtemperature_trips_when_the_sensor_node_reaches_61(pack_temp_C, trips):
    car = create_sr18(initial_temp_C=pack_temp_C)
    car.update(apps_percent=0.0)
    assert car.bms.isShutdown() is trips
