"""Understeer / oversteer maneuvers on the planar car, layered over the existing skidpad pieces.

Nothing in here edits the existing runtime. It imports the plant (PlanarChassis.derivatives
through rk4Step), the skidpad driver and the rear TV controller, and adds:

  CappedRearTV      rear TV whose yaw reference is capped at the friction limit ay_max/u
  runManeuver       runSkidpad's tick loop + disturbances, any start state, optional circle stop
  PerWheelTire      lets one wheel pair run on a different tire (used for a rear grip drop)
  yawRateKick / yawKick / rearGripDrop / rearTorquePulse / liftOff   disturbance builders
  fixedDriver / circleDriver / straightStartState     maneuver set-up helpers

A disturbance is a function of time that returns a dict (empty when inactive):
  base_torque_scale     multiplies the driver's torque before TV sees it (lift-off)
  wheel_torque_add_Nm   [4] added after TV, before the motor limits (rear torque pulse)
  yaw_moment_Nm         added to the external yaw moment, logged on its own channel
  yaw_rate_jump_radps   added to the yaw-rate state at the start of the tick (yaw-rate kick)
  tires                 [4] tire models FL/FR/RL/RR for this tick (rear grip drop)
"""
import copy
import math

import numpy as np

from Functions.rk4 import rk4Step
from Vehicle.Chassis.PlanarChassis import derivatives
from Vehicle.Driver.SkidPadDriverConfig import SkidPadDriverConfig
from Vehicle.VCU.RearTVConfig import RearTVConfig

SKIDPAD_RADIUS_M = 9.125
EVENT_EPS_S = 1e-9          # tick times are accumulated floats, so window edges get a little slack


class CappedRearTV(RearTVConfig):
    """RearTVConfig with r_ref limited to +-ay_max/u (None = exactly the parent controller).

    The steer angle's size only feeds r_ref = u*delta/(L + K*u^2); everything else in the parent
    (turn sign, which rear wheel is inside) uses only its sign. So shrinking |delta| before the
    parent runs caps r_ref exactly, without copying the controller."""

    def __init__(self, *args, ay_max_mps2=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.ay_max_mps2 = ay_max_mps2
        self.r_ref_raw_radps = 0.0   # what r_ref would be with no cap

    @classmethod
    def fromConfig(cls, tv, mode, ay_max_mps2=None):
        return cls(kp_Nm_per_radps=tv.kp_Nm_per_radps, ki_Nm_per_rad=tv.ki_Nm_per_rad,
                   wheelbase_m=tv.wheelbase_m, ref_filter_time_s=tv.ref_filter_time_s,
                   understeer_gradient=tv.understeer_gradient, mode=mode,
                   torque_rate_limit_Nm_per_s=tv.torque_rate_limit_Nm_per_s,
                   ay_max_mps2=ay_max_mps2)

    def _gradient(self, u):
        if callable(self.understeer_gradient):
            return self.understeer_gradient(u)
        return self.understeer_gradient

    def update(self, x, delta_rad, base_wheel_torque_Nm, dt_s):
        if delta_rad == 0:
            self.r_ref_raw_radps = 0.0
            return super().update(x, delta_rad, base_wheel_torque_Nm, dt_s)

        u = x[3]
        denominator = self.wheelbase_m + self._gradient(u) * u * u
        self.r_ref_raw_radps = u * delta_rad / denominator

        delta_used = delta_rad
        if self.ay_max_mps2 is not None and u > 0:
            # |u*delta/denominator| <= ay_max/u  <=>  |delta| <= ay_max*denominator/u^2
            delta_max = self.ay_max_mps2 * denominator / (u * max(u, 1.0))
            delta_used = math.copysign(min(abs(delta_rad), delta_max), delta_rad)
        return super().update(x, delta_used, base_wheel_torque_Nm, dt_s)


class PerWheelTire:
    """Stands in for car.tire inside derivatives(): call k of one derivatives() pass goes to wheel k.

    derivatives() asks the tire for forces once per wheel, FL, FR, RL, RR in that order, and
    perWheelDerivatives() resets the counter before every pass."""

    def __init__(self, tires):
        self.tires = list(tires)
        self.calls = 0

    def calculateForces(self, Fz_N, slip_ratio, slip_angle_rad):
        tire = self.tires[self.calls % 4]
        self.calls += 1
        return tire.calculateForces(Fz_N, slip_ratio, slip_angle_rad)

    def __getattr__(self, name):          # radius_m, peaks, ... come from the FL tire
        if name == "tires":
            raise AttributeError(name)
        return getattr(self.tires[0], name)


def perWheelDerivatives(x, inputs, car):
    car.tire.calls = 0
    return derivatives(x, inputs, car)


def scaledGripTire(tire, scale):
    """Same tire, peak force scaled by `scale`, slip stiffness unchanged (a slippery patch)."""
    return type(tire)(radius_m=tire.radius_m, psi=tire.psi,
                      track_grip_scale=tire.track_grip_scale * scale)


def tireUtilization(tire, Fz_N, Fx_N, Fy_N):
    """How much of the friction ellipse one tire is using: 1.0 = at the limit."""
    if Fz_N <= 0:
        return 1.0
    peak_x = tire.getPeakLongitudinal_N(Fz_N)
    peak_y = tire.getPeakLateral_N(Fz_N)
    return math.hypot(Fx_N / peak_x if peak_x else 0.0, Fy_N / peak_y if peak_y else 0.0)


# ---------------------------------------------------------------- disturbances

def _window(t0_s, duration_s, active):
    def disturbance(t):
        if t0_s - EVENT_EPS_S <= t < t0_s + duration_s - EVENT_EPS_S:
            return active
        return {}
    return disturbance


def yawRateKick(t0_s, jump_radps, control_period_s=0.01):
    """Yaw rate jumps by jump_radps (+ = toward a left turn) at t0: an initial yaw-rate error.
    A moment pulse is mostly soaked up by the tires' yaw damping (~15 ms time constant here),
    so this is the cleaner over-rotation probe."""
    return _window(t0_s, control_period_s, {"yaw_rate_jump_radps": jump_radps})


def yawKick(t0_s, moment_Nm, duration_s=0.1):
    """Extra yaw moment (+ = toward a left turn) for duration_s: an over-rotation impulse."""
    return _window(t0_s, duration_s, {"yaw_moment_Nm": moment_Nm})


def rearGripDrop(car, t0_s, scale, duration_s=1.0):
    """Both rear tires lose (1-scale) of their peak force for duration_s."""
    low = scaledGripTire(car.tire, scale)
    return _window(t0_s, duration_s, {"tires": [car.tire, car.tire, low, low]})


def rearTorquePulse(t0_s, add_Nm, duration_s=0.5):
    """add_Nm extra wheel torque on both rears for duration_s (power oversteer)."""
    return _window(t0_s, duration_s, {"wheel_torque_add_Nm": [0.0, 0.0, add_Nm, add_Nm]})


def liftOff(t0_s, duration_s=0.5):
    """Driver lifts completely for duration_s; TV sees zero base torque."""
    return _window(t0_s, duration_s, {"base_torque_scale": 0.0})


# ---------------------------------------------------------------- set-up helpers

def _driver(car, mode, speed_fn, steer_fn=None, radius_m=SKIDPAD_RADIUS_M):
    return SkidPadDriverConfig(mode=mode, target_speed_mps=speed_fn, delta_rad=steer_fn,
                               radius_m=radius_m, turn="left", lookahead_m=3.0,
                               wheelbase_m=car.wheelbase, speed_kp=30.0, speed_ki=10.0,
                               max_wheeltorque_Nm=248.0, center_m=(0.0, radius_m))


def fixedDriver(car, speed_fn, steer_fn, radius_m=SKIDPAD_RADIUS_M):
    """Open-loop steering steer_fn(t), speed PI to speed_fn(t). Same gains as examples/skidpad.py."""
    return _driver(car, "fixed", speed_fn, steer_fn, radius_m)


def circleDriver(car, speed_fn, radius_m=SKIDPAD_RADIUS_M):
    """Pure pursuit on a left circle of radius_m centred at (0, radius_m)."""
    return _driver(car, "circle", speed_fn, None, radius_m)


def straightStartState(car, speed_mps):
    """Rolling straight along +X at speed_mps, no slip."""
    w = speed_mps / car.wheel_radius
    return [0.0, 0.0, 0.0, speed_mps, 0.0, 0.0, w, w, w, w]


# ---------------------------------------------------------------- runner

def runManeuver(car, driver, tv, run_time_s, x0, center_m=None, radius_m=None, disturbance=None,
                stop_on_radius_error=True, dt_s=0.0005, control_period_s=0.01):
    """runSkidpad's tick loop (same order, same log channels) plus:
    disturbance(t), any start state x0, and the 2 m circle stop only when asked and a centre is given.
    Extra channels: tire_util[4], r_ref_raw_radps, base_torque_Nm, disturbance_yaw_moment_Nm,
    disturbance_active."""
    x = list(x0)
    t = 0.0
    ax, ay = 0.0, 0.0
    log = []
    stop_reason = "time_up"

    steps_per_tick = round(control_period_s / dt_s)
    num_ticks = round(run_time_s / control_period_s)
    rear_half_track = car.getTrack(3) / 2

    for tick in range(num_ticks):
        d = disturbance(t) if disturbance is not None else {}
        if "yaw_rate_jump_radps" in d:
            x[5] += d["yaw_rate_jump_radps"]

        delta, base_torque = driver.update(x, t, control_period_s)
        base_torque *= d.get("base_torque_scale", 1.0)
        d_FL, d_FR = car.steering.getWheelAngles_rad(delta)
        torques, yaw_moment = tv.update(x, delta, base_torque, control_period_s)
        if "wheel_torque_add_Nm" in d:
            torques = [T + extra for T, extra in zip(torques, d["wheel_torque_add_Nm"])]
        torques = car.limits.capWheelTorques_Nm(torques, x[6:10])
        kick = d.get("yaw_moment_Nm", 0.0)

        if "tires" in d:
            plant = copy.copy(car)
            plant.tire = PerWheelTire(d["tires"])
            f = perWheelDerivatives
            tires = d["tires"]
        else:
            plant, f, tires = car, derivatives, [car.tire] * 4

        inputs = {
            "steer_angles_rad": [d_FL, d_FR, 0.0, 0.0],
            "wheel_torques_Nm": torques,
            "external_yaw_moment_Nm": yaw_moment + kick,
            "ax_mps2": ax,
            "ay_mps2": ay,
        }

        for step in range(steps_per_tick):
            x, outputs = rk4Step(x, inputs, plant, dt_s, f=f)
            inputs["ax_mps2"] = outputs["ax_mps2"]
            inputs["ay_mps2"] = outputs["ay_mps2"]
            ax, ay = inputs["ax_mps2"], inputs["ay_mps2"]

        inside = 2 if delta > 0 else 3
        outside = 3 if delta > 0 else 2
        tv_yaw_moment = (outputs["Fx_N"][outside] - outputs["Fx_N"][inside]) * rear_half_track
        motor_power = sum(torques[i] * x[6 + i] for i in range(4))

        if center_m is not None and radius_m is not None:
            radius_error = math.hypot(x[0] - center_m[0], x[1] - center_m[1]) - radius_m
        else:
            radius_error = math.nan

        log.append({
            "time_s": t,
            "X_m": x[0], "Y_m": x[1], "psi_rad": x[2],
            "u_mps": x[3], "v_mps": x[4], "r_radps": x[5],
            "sideslip_rad": math.atan2(x[4], x[3]),
            "ax_mps2": ax,
            "ay_mps2": ay,
            "delta_rad": delta,
            "base_torque_Nm": base_torque,
            "r_ref_radps": tv.r_ref_radps,
            "r_ref_raw_radps": getattr(tv, "r_ref_raw_radps", tv.r_ref_radps),
            "r_ref_filtered_radps": tv.r_ref_filtered_radps if tv.r_ref_filtered_radps is not None else 0.0,
            "yaw_error_radps": tv.yaw_error_radps,
            "inside_cut_Nm": tv.inside_cut_Nm,
            "is_saturated": tv.is_saturated,
            "external_yaw_moment_Nm": yaw_moment,
            "disturbance_yaw_moment_Nm": kick,
            "disturbance_active": bool(d),
            "tv_yaw_moment_Nm": tv_yaw_moment,
            "motor_power_W": motor_power,
            "wheel_torques_Nm": list(torques),
            "Fz_N": list(outputs["Fz_N"]),
            "Fx_N": list(outputs["Fx_N"]),
            "Fy_N": list(outputs["Fy_N"]),
            "slip_ratios": list(outputs["slip_ratios"]),
            "slip_angles_rad": list(outputs["slip_angles_rad"]),
            "tire_util": [tireUtilization(tires[i], outputs["Fz_N"][i], outputs["Fx_N"][i],
                                          outputs["Fy_N"][i]) for i in range(4)],
            "radius_error_m": radius_error,
        })

        if x[3] < 3.0:
            stop_reason = "too slow"
            break
        if stop_on_radius_error and abs(radius_error) > 2.0:   # nan compares False: no centre, no stop
            stop_reason = "off the circle"
            break
        if abs(log[-1]["sideslip_rad"]) > math.radians(20):
            stop_reason = "spun"
            break

        t += control_period_s

    result = {name: np.array([row[name] for row in log]) for name in log[0]}
    result["stop_reason"] = stop_reason
    result["center_m"] = center_m
    return result
