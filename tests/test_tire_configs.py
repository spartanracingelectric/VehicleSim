"""Every tire under Vehicle/Tire/Configs must be a fully populated, sane Dugoff tire.

NOTE: These test cases are AI-generated on purpose, as an independent check on the hand-written model.
"""
import pytest

from Functions.loadConfigs import loadConfigs
from Vehicle.Tire.TireConfig import TireConfig

tires = vars(loadConfigs("Vehicle.Tire.Configs"))


def test_at_least_one_tire_config_exists():
    assert tires


@pytest.mark.parametrize("name", sorted(tires))
class TestTireConfig:
    def test_is_a_tire_config(self, name):
        assert isinstance(tires[name], TireConfig)

    @pytest.mark.parametrize("field", ["radius_m", "longitudinal_stiffness", "lateral_stiffness", "tireroadfriction"])
    def test_parameter_is_set_and_positive(self, name, field):
        value = getattr(tires[name], field)
        assert value is not None, f"{name}.{field} is not set (still None)"
        assert value > 0, f"{name}.{field} must be > 0, got {value}"

    def test_radius_is_a_plausible_size_in_meters(self, name):
        assert 0.1 <= tires[name].radius_m <= 0.6

    def test_produces_nonzero_forces(self, name):
        tire = tires[name]
        tire.calculateTireSlipRatio(11.0, 10.0)
        tire.calculateTireSlipAngle(0.05, 15.0, 0.0)
        tire.calculateCoupling(800.0)
        assert tire.calculateTireLongitudinalForce() != 0.0
        assert tire.calculateTireLateralForce() != 0.0
