"""Every tire under Vehicle/Tire/Configs must be a fully populated, sane tire (Dugoff or Magic Formula),
and every one must work through the shared calls the chassis uses, so any of them can be swapped into the car.

NOTE: These test cases are AI-generated on purpose, as an independent check on the hand-written model.
"""
import math

import pytest

from Functions.loadConfigs import loadConfigs
from Vehicle.Tire.TireConfig import TireConfig
from Vehicle.Tire.MagicFormulaTireConfig import MagicFormulaTireConfig

tires = vars(loadConfigs("Vehicle.Tire.Configs"))
dugoff_tires = {name: tire for name, tire in tires.items() if isinstance(tire, TireConfig)}


def test_at_least_one_tire_config_exists():
    assert tires


@pytest.mark.parametrize("name", sorted(tires))
class TestEveryTire:
    def test_is_a_known_tire_model(self, name):
        assert isinstance(tires[name], (TireConfig, MagicFormulaTireConfig))
        assert tires[name].model_name in ("dugoff", "magic_formula")

    def test_radius_is_a_plausible_size_in_meters(self, name):
        assert 0.1 <= tires[name].radius_m <= 0.6

    def test_produces_nonzero_forces(self, name):
        fx, fy = tires[name].calculateForces(800.0, 0.1, 0.05)
        assert fx > 0.0 and fy > 0.0

    def test_forces_stay_inside_the_friction_limit(self, name):
        tire = tires[name]
        mu = tire.tireroadfriction if isinstance(tire, TireConfig) else tire.mu
        for slip_ratio in (-1.0, -0.3, 0.0, 0.1, 0.5, 2.0):
            for slip_angle in (-0.4, 0.0, 0.05, 0.4):
                fx, fy = tire.calculateForces(800.0, slip_ratio, slip_angle)
                assert math.isfinite(fx) and math.isfinite(fy)
                assert math.hypot(fx, fy) <= mu * 800.0 * (1 + 1e-9)

    def test_grip_limit_flag(self, name):
        tire = tires[name]
        tire.calculateForces(800.0, 0.001, 0.0)
        assert not tire.isAtGripLimit()
        tire.calculateForces(800.0, -1.0, 0.0)
        assert tire.isAtGripLimit()


@pytest.mark.parametrize("name", sorted(dugoff_tires))
class TestDugoffTire:
    @pytest.mark.parametrize("field", ["radius_m", "tireroadfriction"])
    def test_parameter_is_set_and_positive(self, name, field):
        value = getattr(dugoff_tires[name], field)
        assert value is not None, f"{name}.{field} is not set (still None)"
        assert value > 0, f"{name}.{field} must be > 0, got {value}"

    @pytest.mark.parametrize("direction", ["longitudinal", "lateral"])
    def test_stiffness_is_set_one_way_and_positive(self, name, direction):
        fixed = getattr(dugoff_tires[name], f"{direction}_stiffness")
        per_load = getattr(dugoff_tires[name], f"{direction}_stiffness_per_load")
        assert (fixed is None) != (per_load is None), f"{name}: set exactly one of {direction}_stiffness / {direction}_stiffness_per_load"
        value = fixed if fixed is not None else per_load
        assert value > 0, f"{name}.{direction} stiffness must be > 0, got {value}"

    def test_produces_nonzero_forces_one_call_at_a_time(self, name):
        tire = dugoff_tires[name]
        tire.calculateTireSlipRatio(11.0, 10.0)
        tire.calculateTireSlipAngle(0.05, 15.0, 0.0)
        tire.calculateCoupling(800.0)
        assert tire.calculateTireLongitudinalForce() != 0.0
        assert tire.calculateTireLateralForce() != 0.0
