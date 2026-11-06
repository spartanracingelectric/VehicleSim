"""Tests for the four wheel chassis (Vehicle/Chassis/ChassisConfig.py): every wheel on its own, the slip floor and clamp,
no oscillation, load transfer, torque vectoring and per wheel brakes.

NOTE: These test cases are AI-generated on purpose, as an independent check on the hand-written model.
"""
import math

import numpy as np
import pytest

from Functions.constants import G_MPS2
from Vehicle.Chassis.ChassisConfig import ChassisConfig, FL, FR, RL, RR, MAX_SLIP_RATIO
from Vehicle.Tire.Configs.tire1 import tire1

MASS_KG = 255.0
DT_S = 1e-4
TIRES = [tire1.copy().setModel("dugoff"), tire1.copy().setModel("pacejka")]
IDS = ["dugoff", "pacejka"]


def make_chassis(speed_mps=0.0, **kwargs):
    chassis = ChassisConfig(0.0, 0.0, 1.53924, 0.3129, 0.509, 0.225, **kwargs)
    chassis.speed_mps = speed_mps
    return chassis


def run(chassis, tire, seconds, torque=0.0, brake=0.0, steer=0.0):
    rows = []
    for _ in range(round(seconds / DT_S)):
        chassis.update(torque, brake, 0.038, MASS_KG, tire, DT_S, steering_angle_rad=steer)
        rows.append((chassis.speed_mps, *chassis.wheel_speed_radps, *chassis.slip_ratio, chassis.yaw_rate_radps))
    return np.array(rows)


def reversals(x):
    d = np.diff(x)
    d = d[np.abs(d) > 1e-9 * max(1.0, np.abs(x).max())]
    return int(np.sum(np.sign(d[1:]) != np.sign(d[:-1]))) if len(d) > 1 else 0


def test_slip_floor_and_clamp_defaults():
    assert make_chassis().min_slip_speed_mps == 0.1
    assert MAX_SLIP_RATIO == 0.99


@pytest.mark.parametrize("tire", TIRES, ids=IDS)
class TestStability:
    # the floor makes the slip stiff (time constant ~30 us, under the 100 us step), these must still settle, not ring
    @pytest.mark.parametrize("v0, torque, brake", [(0.0, 40.0, 0.0), (0.0, 600.0, 0.0), (0.05, 100.0, 0.0), (0.3, 10.0, 0.0),
                                                   (20.0, 0.0, 2000.0), (2.0, 0.0, 2000.0), (5.0, 0.0, 60.0), (3.0, -30.0, 0.0)])
    def test_no_oscillation(self, tire, v0, torque, brake):
        rows = run(make_chassis(v0), tire, 0.6, torque=torque, brake=brake)
        assert np.isfinite(rows).all()
        assert np.abs(rows[:, 5:9]).max() <= MAX_SLIP_RATIO + 1e-12                 # slip never reaches +-1
        assert reversals(rows[:, 0]) == 0                                           # car speed
        for wheel in range(1, 5):
            assert reversals(rows[:, wheel]) == 0, "wheel speed rang"
        for wheel in range(5, 9):
            assert reversals(rows[:, wheel]) <= 2, "slip ratio rang"             # one smooth bump at most

    def test_same_answer_with_a_finer_step(self, tire):
        # the 100 us step against 25 us: stays close, so the step isn't hiding anything. a launch from 0 is the worst case
        # (~5% slower at 0.4 s, the implicit wheel step trades a little accuracy for never ringing at the 0.1 m/s floor)
        def final_speed(dt):
            chassis = make_chassis(0.0)
            for _ in range(round(0.4 / dt)):
                chassis.update(40.0, 0.0, 0.038, MASS_KG, tire, dt)
            return chassis.speed_mps
        assert final_speed(DT_S) == pytest.approx(final_speed(DT_S / 4), rel=0.08)

    def test_steady_corner_settles_without_yaw_ringing(self, tire):
        rows = run(make_chassis(15.0), tire, 1.5, torque=20.0, steer=math.radians(4.0))
        yaw = rows[:, 9]
        assert yaw[-1] > 0.0
        tail = yaw[len(yaw) // 2:]
        assert reversals(tail) <= 2
        assert tail.max() - tail.min() < 0.15 * abs(yaw[-1])

    def test_slip_filter_can_be_turned_on(self, tire):
        rows = run(make_chassis(0.0, slip_filter_time_constant_s=0.002), tire, 0.3, torque=100.0)
        assert np.isfinite(rows).all() and rows[-1, 0] > 0.0


@pytest.mark.parametrize("tire", TIRES, ids=IDS)
class TestFourWheels:
    def test_straight_line_is_left_right_symmetric(self, tire):
        chassis = make_chassis(10.0)
        run(chassis, tire, 0.3, torque=60.0)
        assert chassis.wheel_speed_radps[FL] == pytest.approx(chassis.wheel_speed_radps[FR])
        assert chassis.wheel_speed_radps[RL] == pytest.approx(chassis.wheel_speed_radps[RR])
        assert chassis.yaw_rate_radps == pytest.approx(0.0, abs=1e-9)

    def test_each_wheel_takes_its_own_torque(self, tire):
        chassis = make_chassis(10.0)
        run(chassis, tire, 0.2, torque=[60.0, 0.0, 0.0, 0.0])
        speeds = chassis.wheel_speed_radps
        assert speeds[FL] > speeds[FR] and speeds[FL] > speeds[RL] and speeds[FL] > speeds[RR]

    def test_wheels_get_separate_tire_models(self, tire):
        chassis = make_chassis(10.0)
        run(chassis, tire, 0.01, torque=[200.0, 0.0, 0.0, 0.0])
        models = chassis.getWheelTires(tire)
        assert len({id(model) for model in models}) == 4
        assert models[FL].tireSlipRatio != models[FR].tireSlipRatio

    def test_more_force_on_the_right_yaws_the_car_left(self, tire):
        # torque vectoring: right wheels push harder than the left ones
        chassis = make_chassis(10.0)
        run(chassis, tire, 0.3, torque=[10.0, 50.0, 10.0, 50.0])
        assert chassis.yaw_rate_radps > 0.0
        chassis = make_chassis(10.0)
        run(chassis, tire, 0.3, torque=[50.0, 10.0, 50.0, 10.0])
        assert chassis.yaw_rate_radps < 0.0

    def test_braking_one_side_yaws_toward_that_side(self, tire):
        chassis = make_chassis(15.0)
        run(chassis, tire, 0.2, brake=[200.0, 0.0, 200.0, 0.0])
        assert chassis.yaw_rate_radps > 0.0     # left brakes harder, car turns left

    def test_cornering_moves_load_to_the_outside_wheels(self, tire):
        chassis = make_chassis(15.0)
        run(chassis, tire, 1.0, steer=math.radians(5.0))      # left turn
        loads = chassis.tire_load_N
        assert loads[FR] > loads[FL] and loads[RR] > loads[RL]
        assert sum(loads) == pytest.approx(MASS_KG * G_MPS2, rel=1e-3)

    def test_braking_moves_load_to_the_front_wheels(self, tire):
        chassis = make_chassis(20.0)
        run(chassis, tire, 0.3, brake=150.0)
        loads = chassis.tire_load_N
        assert min(loads[FL], loads[FR]) > max(loads[RL], loads[RR])

    def test_inner_front_wheel_steers_more(self, tire):
        chassis = make_chassis(10.0)
        run(chassis, tire, 0.01, steer=math.radians(10.0))
        assert chassis.wheel_steer_rad[FL] > chassis.wheel_steer_rad[FR] > 0.0

    def test_wrong_number_of_torques_is_an_error(self, tire):
        with pytest.raises(ValueError):
            make_chassis(1.0).update([1.0, 2.0], 0.0, 0.0, MASS_KG, tire, DT_S)

    def test_a_locked_wheel_stays_locked_and_never_goes_backwards(self, tire):
        chassis = make_chassis(15.0)
        rows = run(chassis, tire, 0.5, brake=[2000.0, 0.0, 0.0, 0.0])
        assert rows[:, 1].min() >= 0.0
        assert chassis.wheel_speed_radps[FL] == 0.0
        assert chassis.slip_ratio[FL] == pytest.approx(-MAX_SLIP_RATIO)


def test_legacy_tire_objects_still_work_and_get_a_copy_per_wheel():
    from Vehicle.Tire.TireConfig import TireConfig
    tire = TireConfig(0.2, longitudinal_stiffness_per_load=20.0, lateral_stiffness_per_load=30.0, tireroadfriction=1.2)
    chassis = make_chassis(10.0)
    run(chassis, tire, 0.05, torque=30.0)
    assert len({id(model) for model in chassis.getWheelTires(tire)}) == 4
