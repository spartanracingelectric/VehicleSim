import numpy as np
import pytest

from Vehicle.Battery.Configs.sr17 import create_sr17_battery, create_sr17_thermal_model

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
