"""Tests for the brakes (Vehicle/Brake/BrakeConfig.py) and the wheel slip chassis (Vehicle/Chassis/ChassisConfig.py).

NOTE: These test cases are AI-generated on purpose, as an independent check on the hand-written model.

Brake reference: line pressure = pedal % * max pedal force * pedal ratio * bias / master cylinder area
                 wheel torque  = 2 * pad mu * pressure * piston area per side * rotor radius
"""
import math
from copy import deepcopy

import pytest

from Functions.constants import G_MPS2
from Vehicle.Brake.BrakeConfig import BrakeConfig
from Vehicle.Chassis.ChassisConfig import ChassisConfig
from Vehicle.Tire.Configs.tire1 import tire1
from Vehicle.Tire.Configs.tire1_mf import tire1_mf
from Tests.scenario import Scenario, runScenario
from Tests.pedals import hold, steps

TIRES = [tire1, tire1_mf]
TIRE_IDS = ["dugoff", "magic_formula"]


def make_brake(**overrides):
    kwargs = dict(mass_kg=10, max_pedal_force_N=500.0, pedal_ratio=5.0, front_bias=0.65,
                  front_master_cylinder_bore_m=0.015875, rear_master_cylinder_bore_m=0.015875,
                  front_caliper_piston_diameter_m=0.025, front_caliper_pistons_per_side=2,
                  rear_caliper_piston_diameter_m=0.022, rear_caliper_pistons_per_side=1,
                  pad_mu=0.45, front_rotor_radius_m=0.085, rear_rotor_radius_m=0.080)
    kwargs.update(overrides)
    return BrakeConfig(**kwargs)


def area(diameter_m):
    return math.pi * diameter_m ** 2 / 4


# ---------------------------------------------------------------- brakes on their own
class TestBrake:
    def test_no_pedal_no_torque(self):
        brake = make_brake()
        brake.update(0.0)
        assert brake.getFrontWheelTorque_Nm() == 0.0 and brake.getRearWheelTorque_Nm() == 0.0

    def test_matches_the_hydraulics_by_hand(self):
        brake = make_brake()
        brake.update(40.0)
        push_rod_N = 0.40 * 500.0 * 5.0
        front_pressure = push_rod_N * 0.65 / area(0.015875)
        rear_pressure = push_rod_N * 0.35 / area(0.015875)
        assert brake.getFrontPressure_Pa() == pytest.approx(front_pressure)
        assert brake.getRearPressure_Pa() == pytest.approx(rear_pressure)
        assert brake.getFrontWheelTorque_Nm() == pytest.approx(2 * 0.45 * front_pressure * 2 * area(0.025) * 0.085)
        assert brake.getRearWheelTorque_Nm() == pytest.approx(2 * 0.45 * rear_pressure * 1 * area(0.022) * 0.080)

    def test_torque_is_linear_in_pedal_and_capped_at_100(self):
        brake = make_brake()
        brake.update(25.0)
        quarter = brake.getFrontWheelTorque_Nm()
        brake.update(100.0)
        full = brake.getFrontWheelTorque_Nm()
        brake.update(150.0)
        assert full == pytest.approx(4 * quarter)
        assert brake.getFrontWheelTorque_Nm() == pytest.approx(full)
        brake.update(-10.0)
        assert brake.getFrontWheelTorque_Nm() == 0.0

    def test_bias_moves_torque_between_axles(self):
        all_front = make_brake(front_bias=1.0)
        all_front.update(50.0)
        assert all_front.getRearWheelTorque_Nm() == 0.0 and all_front.getFrontWheelTorque_Nm() > 0

    @pytest.mark.parametrize("bad", [dict(front_bias=1.2), dict(front_bias=-0.1), dict(pad_mu=-0.1),
                                     dict(pedal_ratio=0.0), dict(front_master_cylinder_bore_m=0.0),
                                     dict(rear_rotor_radius_m=-0.08)])
    def test_bad_values_raise(self, bad):
        with pytest.raises(ValueError):
            make_brake(**bad)


# ---------------------------------------------------------------- chassis on its own (no powertrain)
MASS_KG = 255.0
DT_S = 1e-4
STOPPED_MPS = 1e-3    # the last bit of speed fades out smoothly (min_slip_speed_mps), under 1 mm/s is stopped


def make_chassis(speed_mps=0.0):
    chassis = ChassisConfig(drag_area_m2=0.0, rolling_resistance_coeff=0.0, wheelbase_m=1.53924, cg_height_m=0.3129,
                            rear_weight_fraction=0.509, wheel_inertia_kgm2=0.225)
    chassis.speed_mps = speed_mps
    return chassis


def run_chassis(chassis, tire, seconds, wheel_torque_Nm=0.0, front_brake_Nm=0.0, rear_brake_Nm=0.0):
    history = []
    for _ in range(round(seconds / DT_S)):
        chassis.update(wheel_torque_Nm, front_brake_Nm, rear_brake_Nm, 0.038, MASS_KG, tire, DT_S)
        history.append((chassis.speed_mps, chassis.front_wheel_speed_radps, chassis.rear_wheel_speed_radps,
                        chassis.front_slip_ratio, chassis.rear_slip_ratio))
    return history


@pytest.mark.parametrize("tire", TIRES, ids=TIRE_IDS)
class TestChassis:
    def test_parked_with_no_torque_stays_parked(self, tire):
        chassis = make_chassis()
        run_chassis(chassis, tire, 0.2, front_brake_Nm=500.0, rear_brake_Nm=500.0)
        assert chassis.speed_mps == 0.0 and chassis.front_wheel_speed_radps == 0.0

    def test_rolling_with_no_torque_has_no_slip_and_keeps_its_speed(self, tire):
        chassis = make_chassis(speed_mps=20.0)
        run_chassis(chassis, tire, 0.2)
        assert chassis.speed_mps == pytest.approx(20.0)
        assert chassis.front_slip_ratio == pytest.approx(0.0, abs=1e-9)

    def test_wheels_start_rolling_with_the_car(self, tire):
        chassis = make_chassis(speed_mps=15.0)
        assert chassis.getWheelSpeed_radps(tire.radius_m) == pytest.approx(15.0 / tire.radius_m)
        run_chassis(chassis, tire, DT_S)
        assert chassis.front_wheel_speed_radps == pytest.approx(15.0 / tire.radius_m)

    def test_small_torque_small_slip_and_the_car_speeds_up(self, tire):
        chassis = make_chassis(speed_mps=10.0)
        history = run_chassis(chassis, tire, 0.5, wheel_torque_Nm=40.0)
        assert chassis.speed_mps > 10.0
        assert 0 < chassis.rear_slip_ratio < 0.05 and not chassis.isTractionLimited()
        # rotating wheels soak up some of the push: a < 4 * T / (r m)
        assert (chassis.speed_mps - 10.0) / 0.5 < 4 * 40.0 / (tire.radius_m * MASS_KG)
        assert all(row[0] >= history[0][0] for row in history)

    def test_too_much_torque_spins_the_wheels(self, tire):
        chassis = make_chassis(speed_mps=5.0)
        run_chassis(chassis, tire, 0.3, wheel_torque_Nm=600.0)
        assert chassis.front_slip_ratio > 0.3
        assert chassis.isTractionLimited()

    def test_hard_brake_locks_the_wheels_and_stops_the_car(self, tire):
        chassis = make_chassis(speed_mps=20.0)
        history = run_chassis(chassis, tire, 3.0, front_brake_Nm=2000.0, rear_brake_Nm=2000.0)
        assert chassis.speed_mps < STOPPED_MPS
        assert all(row[1] >= 0.0 and row[2] >= 0.0 for row in history), "a wheel spun backwards"
        assert any(row[3] == -1.0 for row in history), "front never locked"
        assert all(b[0] <= a[0] for a, b in zip(history, history[1:])), "the car sped up while braking"

    def test_locked_stop_takes_about_the_sliding_distance(self, tire):
        chassis = make_chassis(speed_mps=20.0)
        run_chassis(chassis, tire, 3.0, front_brake_Nm=2000.0, rear_brake_Nm=2000.0)
        sliding_mu = -tire.calculateForces(1000.0, -1.0, 0.0)[0] / 1000.0    # the tire's own locked-wheel grip
        expected_m = 20.0 ** 2 / (2 * sliding_mu * G_MPS2)
        assert chassis.distance_m == pytest.approx(expected_m, rel=0.1)

    def test_braking_moves_load_to_the_front(self, tire):
        chassis = make_chassis(speed_mps=20.0)
        run_chassis(chassis, tire, 0.3, front_brake_Nm=200.0, rear_brake_Nm=60.0)
        assert chassis.front_tire_load_N > chassis.rear_tire_load_N
        total_N = 2 * (chassis.front_tire_load_N + chassis.rear_tire_load_N)
        assert total_N == pytest.approx(MASS_KG * G_MPS2)


# ---------------------------------------------------------------- the whole car through the scenario runner
def first_time(result, flags):
    return result.time_s[flags.argmax()] if flags.any() else None


# a wheel locked while the car is really moving (at the very end every wheel stops with the car, that doesn't count)
def locked(result, slip_ratio):
    return (slip_ratio <= -0.999) & (result.speed_mps > 3.0)


def test_full_car_hard_brake_from_60_kph():
    result = runScenario(Scenario("hard brake from 60 km/h", pedal=hold(0), brake=hold(100), run_time_s=1.5, start_soc=0.95,
                                  start_speed_kph=60))
    assert result.speed_mps[-1] < STOPPED_MPS
    assert 7.0 < result.distance_m[-1] < 15.0
    assert result.bps_percent[0] == 100.0 and result.front_brake_pressure_Pa[0] > result.rear_brake_pressure_Pa[0]
    front_lock = first_time(result, locked(result, result.front_slip_ratio))
    rear_lock = first_time(result, locked(result, result.rear_slip_ratio))
    assert front_lock is not None and rear_lock is not None and front_lock <= rear_lock, "rears locked first"


def test_full_car_light_brake_does_not_lock():
    result = runScenario(Scenario("light brake from 60 km/h", pedal=hold(0), brake=steps((0.0, 30)), run_time_s=0.5, start_soc=0.95,
                                  start_speed_kph=60))
    assert result.speed_mps[-1] < result.speed_mps[0]
    assert result.front_slip_ratio.min() > -0.2 and result.rear_slip_ratio.min() > -0.2
    assert not result.traction_limited.any()


def test_full_car_runs_on_the_magic_formula_tire():
    from Vehicle.MainConfigs.SR18 import SR18
    car = deepcopy(SR18)
    car.tire = tire1_mf
    result = runScenario(Scenario("magic formula part throttle", pedal=hold(30), run_time_s=0.5, vehicle=car))
    assert result.tire_model == "magic_formula"
    assert result.speed_mps[-1] > 0.0
