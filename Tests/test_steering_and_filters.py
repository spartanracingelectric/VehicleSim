import math

import pytest

from Functions.FirstOrderFilter import FirstOrderFilter
from Vehicle.Chassis.ChassisConfig import ChassisConfig
from Vehicle.Chassis.SteeringConfig import SteeringConfig
from Vehicle.Tire.TireConfig import TireConfig
from Vehicle.Tire.MagicFormulaTireConfig import MagicFormulaTireConfig
from Vehicle.VCU.TorqueVectoringConfig import TorqueVectoringConfig


def make_dugoff(tau=0.0):
    return TireConfig(radius_m=0.2, longitudinal_stiffness_per_load=20.0,
                      lateral_stiffness_per_load=30.0, tireroadfriction=1.2,
                      force_filter_time_constant_s=tau)


def make_pacejka(tau=0.0):
    return MagicFormulaTireConfig(radius_m=0.2, mu=1.2, C_long=1.5, E_long=0.0,
                                  long_stiff_per_load=20.0, C_lat=1.3, E_lat=0.0,
                                  lat_stiff_per_load=30.0,
                                  force_filter_time_constant_s=tau)


class TestFirstOrderFiltering:
    def test_filter_has_expected_backward_euler_step(self):
        filter_ = FirstOrderFilter(0.1)
        assert filter_.update(0.0, 0.01) == 0.0
        assert filter_.update(1.0, 0.01) == pytest.approx(1.0 / 11.0)

    @pytest.mark.parametrize("tire_factory", [make_dugoff, make_pacejka])
    def test_both_tire_models_filter_force(self, tire_factory):
        tire = tire_factory(tau=0.1)
        unfiltered = tire_factory(tau=0.0).calculateForces(1000.0, 0.2, 0.0)[0]
        tire.calculateForces(1000.0, 0.0, 0.0, dt_s=0.01)
        filtered = tire.calculateForces(1000.0, 0.2, 0.0, dt_s=0.01)[0]
        assert 0.0 < filtered < unfiltered

    @pytest.mark.parametrize("tire_factory", [make_dugoff, make_pacejka])
    def test_filter_requires_timestep_when_enabled(self, tire_factory):
        with pytest.raises(ValueError):
            tire_factory(tau=0.1).calculateForces(1000.0, 0.1, 0.0)


class TestSteeringAndTorqueVectoring:
    def test_ackermann_inner_wheel_has_more_angle(self):
        steering = SteeringConfig(wheelbase_m=1.5, front_track_m=1.2)
        left, right = steering.wheel_angles(math.radians(10.0))
        assert left > right > 0.0
        assert steering.wheel_angles(-math.radians(10.0))[0] < 0.0

    def test_torque_vectoring_preserves_total_and_requests_yaw(self):
        allocator = TorqueVectoringConfig(track_m=1.2, wheel_radius_m=0.2)
        left, right = allocator.allocate(100.0, 30.0)
        assert left + right == pytest.approx(100.0)
        assert right > left

    def test_steering_generates_lateral_motion_and_yaw(self):
        chassis = ChassisConfig(drag_area_m2=0.0, rolling_resistance_coeff=0.0,
                                wheelbase_m=1.5, cg_height_m=0.3,
                                rear_weight_fraction=0.5, wheel_inertia_kgm2=0.2)
        chassis.speed_mps = 15.0
        tire = make_dugoff()
        for _ in range(1000):
            chassis.update(0.0, 0.0, 0.0, 0.0, 250.0, tire, 1e-4,
                           steering_angle_rad=math.radians(5.0))
        assert chassis.yaw_rate_radps > 0.0
        assert chassis.y_m > 0.0
        assert chassis.steering_angle_rad == pytest.approx(math.radians(5.0))
