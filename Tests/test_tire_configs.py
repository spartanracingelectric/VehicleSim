"""Every tire under Vehicle/Tire/Configs must be a fully populated, sane tire that works with both models (dugoff and
pacejka), and every model must work through the shared calls the chassis uses, so either can be swapped into the car.

NOTE: These test cases are AI-generated on purpose, as an independent check on the hand-written model.
"""
import math

import pytest

from Functions.loadConfigs import loadConfigs
from Vehicle.Tire.Tire import Tire
from Vehicle.Tire.TireConfig import TireConfig
from Vehicle.Tire.MagicFormulaTireConfig import MagicFormulaTireConfig

tires = vars(loadConfigs("Vehicle.Tire.Configs"))


def test_at_least_one_tire_config_exists():
    assert tires


@pytest.mark.parametrize("name", sorted(tires))
class TestEveryTireConfig:
    def test_is_a_tire(self, name):
        assert isinstance(tires[name], Tire)

    def test_radius_is_a_plausible_size_in_meters(self, name):
        assert 0.1 <= tires[name].radius_m <= 0.6

    @pytest.mark.parametrize("field", ["mu", "long_stiff_per_load", "lat_stiff_per_load"])
    def test_parameter_is_set_and_positive(self, name, field):
        value = getattr(tires[name], field)
        assert value is not None and value > 0, f"{name}.{field} must be set and > 0, got {value}"

    def test_model_flag_picks_the_model(self, name):
        tire = tires[name].copy()
        assert isinstance(tire.setModel("dugoff").createModel(), TireConfig)
        assert isinstance(tire.setModel("pacejka").createModel(), MagicFormulaTireConfig)
        with pytest.raises(ValueError):
            tire.setModel("nonsense")

    def test_wheels_get_their_own_models(self, name):
        models = tires[name].createWheelModels(4)
        assert len({id(model) for model in models}) == 4


@pytest.mark.parametrize("model", Tire.MODELS)
@pytest.mark.parametrize("name", sorted(tires))
class TestEveryModel:
    def make(self, name, model):
        return tires[name].copy().setModel(model).createModel()

    def test_produces_nonzero_forces(self, name, model):
        fx, fy = self.make(name, model).calculateForces(800.0, 0.1, 0.05)
        assert fx > 0.0 and fy > 0.0

    def test_forces_stay_inside_the_friction_limit(self, name, model):
        tire = self.make(name, model)
        mu = tires[name].mu
        for slip_ratio in (-1.0, -0.99, -0.3, 0.0, 0.1, 0.5, 0.99):
            for slip_angle in (-0.4, 0.0, 0.05, 0.4):
                fx, fy = tire.calculateForces(800.0, slip_ratio, slip_angle)
                assert math.isfinite(fx) and math.isfinite(fy)
                assert math.hypot(fx, fy) <= mu * 800.0 * (1 + 1e-9)

    def test_grip_limit_flag(self, name, model):
        tire = self.make(name, model)
        tire.calculateForces(800.0, 0.001, 0.0)
        assert not tire.isAtGripLimit()
        tire.calculateForces(800.0, -0.99, 0.0)
        assert tire.isAtGripLimit()

    def test_both_models_agree_at_small_slip(self, name, model):
        dugoff = self.make(name, "dugoff").calculateForces(800.0, 0.002, 0.002)
        pacejka = self.make(name, "pacejka").calculateForces(800.0, 0.002, 0.002)
        assert pacejka[0] == pytest.approx(dugoff[0], rel=0.03)
        assert pacejka[1] == pytest.approx(dugoff[1], rel=0.03)

    def test_peeking_at_the_forces_leaves_the_filter_alone(self, name, model):
        tire = tires[name].copy()
        tire.force_filter_time_constant_s = 0.01
        a = tire.setModel(model).createModel()
        b = tire.setModel(model).createModel()
        a.calculateForces(800.0, 0.1, 0.0, dt_s=1e-4)
        b.calculateForces(800.0, 0.1, 0.0, dt_s=1e-4)
        b.calculateForces(800.0, 0.9, 0.0, dt_s=1e-4, apply_filter=False)    # a look, not a step
        assert a.calculateForces(800.0, 0.2, 0.0, dt_s=1e-4) == b.calculateForces(800.0, 0.2, 0.0, dt_s=1e-4)


def test_dugoff_one_call_at_a_time_still_works():
    tire = tires["tire1"].copy().setModel("dugoff").createModel()
    tire.calculateTireSlipRatio(11.0, 10.0)
    tire.calculateTireSlipAngle(0.05, 15.0, 0.0)
    tire.calculateCoupling(800.0)
    assert tire.calculateTireLongitudinalForce() != 0.0
    assert tire.calculateTireLateralForce() != 0.0
