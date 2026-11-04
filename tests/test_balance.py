import copy
import math

import numpy as np

from Vehicle import create_skidpad
from Vehicle.VCU.Configs.reartv1 import reartv1
from Vehicle.VCU.RearTVConfig import RearTVConfig
from Vehicle.Simulation.skidpad import runSkidpad
from Vehicle.Simulation import balance as sim
from Data import balanceAnalysis as an

R = sim.SKIDPAD_RADIUS_M


def circleRun(car, tv, seconds=2.0, disturbance=None):
    driver = sim.circleDriver(car, lambda t: 6.0 + 0.5 * (t // 4))
    x0, c = car.getStartStateOnCircle(6.0, R, "left")
    return sim.runManeuver(car, driver, tv, seconds, x0, c, R, disturbance=disturbance)


def test_runner_matches_runSkidpad():
    car = create_skidpad()
    ref = runSkidpad(car, sim.circleDriver(car, lambda t: 6.0 + 0.5 * (t // 4)), copy.deepcopy(reartv1),
                     2.0, start_speed_mps=6.0)
    new = circleRun(car, sim.CappedRearTV.fromConfig(reartv1, "inside_cut"))
    assert ref["stop_reason"] == new["stop_reason"]
    for name in ["X_m", "Y_m", "r_radps", "u_mps", "delta_rad", "inside_cut_Nm", "Fy_N", "radius_error_m"]:
        assert np.array_equal(ref[name], new[name]), name


def test_same_tire_on_every_wheel_is_bit_identical():
    car = create_skidpad()
    a = circleRun(car, sim.CappedRearTV.fromConfig(reartv1, "inside_cut"))
    b = circleRun(car, sim.CappedRearTV.fromConfig(reartv1, "inside_cut"),
                  disturbance=lambda t: {"tires": [car.tire] * 4})
    for name in ["X_m", "r_radps", "Fy_N", "Fx_N"]:
        assert np.array_equal(a[name], b[name]), name


def test_per_wheel_tire_goes_to_the_right_wheel():
    car = create_skidpad()

    class Tagged:
        def __init__(self, k):
            self.k = k

        def calculateForces(self, Fz_N, slip_ratio, slip_angle_rad):
            return float(self.k), 0.0

    plant = copy.copy(car)
    plant.tire = sim.PerWheelTire([Tagged(k) for k in range(4)])
    x0, _ = car.getStartStateOnCircle(6.0, R, "left")
    inputs = {"steer_angles_rad": [0, 0, 0, 0], "wheel_torques_Nm": [0] * 4, "external_yaw_moment_Nm": 0.0,
              "ax_mps2": 0.0, "ay_mps2": 0.0}
    for _ in range(3):                                    # counter resets every pass
        _, outputs = sim.perWheelDerivatives(x0, inputs, plant)
        assert outputs["Fx_N"] == [0.0, 1.0, 2.0, 3.0]


def test_scaled_grip_lowers_peak_not_stiffness():
    car = create_skidpad()
    low = sim.scaledGripTire(car.tire, 0.8)
    assert abs(low.getPeakLateral_N(800.0) / car.tire.getPeakLateral_N(800.0) - 0.8) < 1e-9
    small = math.radians(0.2)                             # linear region: same force
    assert abs(low.calculateForces(800.0, 0.0, small)[1] / car.tire.calculateForces(800.0, 0.0, small)[1] - 1) < 0.01


def stateFor(u, r):
    return [0, 0, 0, u, 0, r, 0, 0, 0, 0]


def test_cap_none_is_the_parent_controller():
    plain = RearTVConfig(kp_Nm_per_radps=200.0, ki_Nm_per_rad=0.0, wheelbase_m=1.53924,
                         ref_filter_time_s=0.05, understeer_gradient=0.0, mode="inside_cut")
    capped = sim.CappedRearTV(kp_Nm_per_radps=200.0, ki_Nm_per_rad=0.0, wheelbase_m=1.53924,
                              ref_filter_time_s=0.05, understeer_gradient=0.0, mode="inside_cut")
    for u, r, d in [(11.588, 1.10, 0.16868), (11.588, -1.10, -0.16868), (8.0, 0.5, 0.3)]:
        assert plain.update(stateFor(u, r), d, 100.0, 0.01) == capped.update(stateFor(u, r), d, 100.0, 0.01)
        assert plain.r_ref_radps == capped.r_ref_radps == capped.r_ref_raw_radps


def test_cap_limits_r_ref_to_friction_and_keeps_the_turn_side():
    u, ay_max = 12.0, 14.0
    for sign, inside in ((1, 2), (-1, 3)):
        tv = sim.CappedRearTV(kp_Nm_per_radps=200.0, ki_Nm_per_rad=0.0, wheelbase_m=1.53924,
                              ref_filter_time_s=0.05, understeer_gradient=0.0, mode="inside_cut",
                              ay_max_mps2=ay_max)
        torques, _ = tv.update(stateFor(u, sign * 0.9), sign * math.radians(28), 100.0, 0.01)
        assert abs(abs(tv.r_ref_radps) - ay_max / u) < 1e-9
        assert abs(tv.r_ref_raw_radps) > 3.5                       # what the uncapped one would chase
        assert torques[inside] < 100.0 and all(torques[i] == 100.0 for i in range(4) if i != inside)


def test_kus_estimator_recovers_a_known_gradient():
    L, K = 1.53, 0.0012
    ay = np.linspace(0.5, 12.0, 400)
    u = np.sqrt(ay * R)
    r = u / R
    delta = np.arctan(L / R) + K * ay
    result = {"u_mps": u, "r_radps": r, "ay_mps2": ay, "delta_rad": delta, "time_s": np.linspace(1, 20, 400)}
    assert abs(an.kusLinear(result, L, 2.0, 10.0) - K) < 1e-9
    c, local = an.kusLocal(result, L)
    assert len(c) > 10 and np.nanmax(np.abs(local - K)) < 1e-9
    offset = dict(result, delta_rad=delta + 0.004)          # constant offset: slope ignores it
    assert abs(an.kusLinear(offset, L, 2.0, 10.0) - K) < 1e-9


def test_settling_and_step_metrics_on_a_damped_sine():
    t = np.arange(0, 6, 0.01)
    y = 1 - np.exp(-3 * (t - 1).clip(0)) * np.cos(8 * (t - 1).clip(0))
    y[t < 1] = 0.0
    settle = an.settlingTime(t, y, 1.0, 0.05, 1.0)
    expected = math.log(1 / 0.05) / 3                      # envelope inside the band
    assert 0.6 * expected < settle <= expected + 0.02
    zeros = np.zeros_like(t)
    met = an.stepMetrics({"time_s": t, "r_radps": y, "sideslip_rad": zeros, "inside_cut_Nm": zeros,
                          "r_ref_filtered_radps": y, "stop_reason": "time_up"}, 1.0)
    assert 20 < met["overshoot_pct"] < 40
    assert abs(met["peak_s"] - (math.pi - math.atan(3 / 8)) / 8) < 0.02    # damped peak, not pi/8


def test_straight_start_stays_straight():
    car = create_skidpad()
    driver = sim.fixedDriver(car, lambda t: 10.0, lambda t: 0.0)
    r = sim.runManeuver(car, driver, sim.CappedRearTV.fromConfig(reartv1, "inside_cut"), 2.0,
                        sim.straightStartState(car, 10.0), stop_on_radius_error=False)
    assert r["stop_reason"] == "time_up"
    assert np.max(np.abs(r["r_radps"])) < 1e-9 and np.max(np.abs(r["Y_m"])) < 1e-9
    assert np.all(np.isnan(r["radius_error_m"]))


def test_sub_limit_yaw_kick_settles():
    car = create_skidpad()
    u = 8.0
    steer = car.wheelbase / R
    driver = sim.fixedDriver(car, lambda t: u, lambda t: steer)
    x0, c = car.getStartStateOnCircle(u, R, "left")
    r = sim.runManeuver(car, driver, sim.CappedRearTV.fromConfig(reartv1, "off"), 4.0, x0, c, R,
                        disturbance=sim.yawRateKick(2.0, 0.3), stop_on_radius_error=False)
    met = an.disturbanceMetrics(r, 2.0, 0.1)
    assert r["stop_reason"] == "time_up"
    # +0.3 rad/s lands at the start of a 10 ms tick; the log is at its end, after ~15 ms-time-constant decay
    assert 0.1 < met["yaw_overshoot_radps"] < 0.3
    assert met["settle_s"] < 1.5


def test_yaw_moment_pulse_is_mostly_damped_by_the_tires():
    # why the O2 kick is a yaw-rate jump: 345 Nm for 0.1 s would be +0.3 rad/s with no tires,
    # but the tires' yaw damping holds it to a fraction of that
    car = create_skidpad()
    steer = car.wheelbase / R
    driver = sim.fixedDriver(car, lambda t: 8.0, lambda t: steer)
    x0, c = car.getStartStateOnCircle(8.0, R, "left")
    r = sim.runManeuver(car, driver, sim.CappedRearTV.fromConfig(reartv1, "off"), 3.0, x0, c, R,
                        disturbance=sim.yawKick(2.0, 345.0), stop_on_radius_error=False)
    assert an.disturbanceMetrics(r, 2.0, 0.1)["yaw_overshoot_radps"] < 0.15
