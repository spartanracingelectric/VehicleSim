"""Steady-state 3-DOF (longitudinal/lateral/yaw) yaw-plane cornering solver.

Unlike Skidpad.py's corner_speed_feasible (which assumes the required lateral force
splits between axles by static weight distribution - a simplification, not a solve),
this actually solves the bicycle-model equilibrium: given a target speed and radius,
find the body sideslip angle and front steer angle that balance lateral force and yaw
moment, using each corner's real slip angle and the car's actual Pacejka tire model
(SR_17.tire_forces / SR_18.tire_forces) rather than a flat or per-axle-assumed split.

Model: a 2-axle bicycle model extended with per-corner (not just per-axle) normal load
from Skidpad.axle_normal_loads, so load transfer between the outer and inner tire on
each axle is real, but both tires on an axle are still assigned the same slip angle
(no separate per-wheel steer/track-width slip angle correction). Rear axle is assumed
undriven/unsteered for the lateral solve - kappa=0 at every corner, i.e. this solves
the pure-cornering (not combined-slip) equilibrium, matching how corner_speed_feasible
already treats the longitudinal drivetrain budget as a separate check layered on top.

Equations (steady state: vx_dot = vy_dot = r_dot = 0, standard body-frame bicycle
kinematics):
    m*vx*r = Fy_front_vehicle + Fy_rear_vehicle          (lateral force balance)
    0 = a*Fy_front_vehicle - b*Fy_rear_vehicle             (yaw moment balance about CG)
where r = vx/radius (prescribed by the circular path), a/b are the CG-to-front/rear
axle distances (from static_rear_bias and wheelbase), and slip angles are:
    alpha_front = delta - atan2(vy + a*r, vx)
    alpha_rear  = -atan2(vy - b*r, vx)
Solved for (delta, vy) via scipy.optimize.fsolve.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "Sims"))  # Skidpad.py

import numpy as np
from scipy.optimize import fsolve
from Skidpad import axle_normal_loads

MAX_STEER_RAD = np.deg2rad(35.0)  # generous typical FSAE steering-lock bound, used only
                                    # as a sanity cap on the solve - not a measured car spec

def _cg_distances(car):
    a = car.static_rear_bias * car.wheelbase        # CG to front axle
    b = (1.0 - car.static_rear_bias) * car.wheelbase  # CG to rear axle
    return a, b

def solve_steady_state_cornering(car, vx, radius, turn_direction=1.0,
                                  delta_guess=0.05, vy_guess=0.0):
    """Solves the yaw-plane equilibrium at speed vx (m/s) and radius (m).

    Returns a dict: converged (bool, both the solver's own convergence and delta
    within MAX_STEER_RAD), delta (rad), vy (m/s), alpha_front/alpha_rear (rad),
    fy_front/fy_rear (N, vehicle-frame), fz_front_outer/inner/rear_outer/inner (N),
    residual (max abs of the two equations, N and N*m respectively - near zero at a
    good solution).
    """
    a, b = _cg_distances(car)
    r = turn_direction * vx / radius if radius > 0 else 0.0

    fz_fo, fz_fi, fz_ro, fz_ri = axle_normal_loads(car, vx, radius)

    def equations(x):
        delta, vy = x
        alpha_f = delta - np.arctan2(vy + a * r, max(vx, 1e-6))
        alpha_r = -np.arctan2(vy - b * r, max(vx, 1e-6))

        _, fy_fo = car.tire_forces(fz_fo, 0.0, alpha_f)
        _, fy_fi = car.tire_forces(fz_fi, 0.0, alpha_f)
        fy_front_tire = fy_fo + fy_fi

        _, fy_ro = car.tire_forces(fz_ro, 0.0, alpha_r)
        _, fy_ri = car.tire_forces(fz_ri, 0.0, alpha_r)
        fy_rear_tire = fy_ro + fy_ri

        fy_front_veh = fy_front_tire * np.cos(delta)
        fy_rear_veh = fy_rear_tire

        eq_lat = fy_front_veh + fy_rear_veh - car.mass * vx * r
        eq_yaw = a * fy_front_veh - b * fy_rear_veh
        return [eq_lat, eq_yaw]

    sol, infodict, ier, msg = fsolve(equations, [delta_guess, vy_guess], full_output=True)
    delta, vy = sol
    residual = float(np.max(np.abs(infodict['fvec'])))
    converged = (ier == 1) and (abs(delta) <= MAX_STEER_RAD) and (residual < 1.0)

    alpha_f = delta - np.arctan2(vy + a * r, max(vx, 1e-6))
    alpha_r = -np.arctan2(vy - b * r, max(vx, 1e-6))
    _, fy_fo = car.tire_forces(fz_fo, 0.0, alpha_f)
    _, fy_fi = car.tire_forces(fz_fi, 0.0, alpha_f)
    _, fy_ro = car.tire_forces(fz_ro, 0.0, alpha_r)
    _, fy_ri = car.tire_forces(fz_ri, 0.0, alpha_r)

    return {
        'converged': converged, 'delta': delta, 'vy': vy, 'r': r,
        'alpha_front': alpha_f, 'alpha_rear': alpha_r,
        'fy_front': (fy_fo + fy_fi) * np.cos(delta), 'fy_rear': fy_ro + fy_ri,
        'fz_front_outer': fz_fo, 'fz_front_inner': fz_fi,
        'fz_rear_outer': fz_ro, 'fz_rear_inner': fz_ri,
        'residual': residual,
    }

def max_corner_speed_yaw_plane(car, radius, v_search_max=60.0, dv=0.05):
    """Sweeps speed upward until the yaw-plane equilibrium stops converging (delta/vy
    blow up or the solve fails) or the drivetrain can no longer sustain the speed - the
    same drivetrain/power budget check corner_speed_feasible already does, layered on
    top of this function's real lateral/yaw solve instead of the axle-split
    approximation. Coarser dv than Skidpad's own bisection (which runs per-waypoint in
    Endurance/Autocross) since a nonlinear solve happens at every step; fine for a
    single fixed-radius scenario like the skidpad event.
    """
    from Skidpad import available_forces
    best_v = 0.0
    best_result = None
    delta_guess, vy_guess = 0.0, 0.0
    for v in np.arange(dv, v_search_max, dv):
        result = solve_steady_state_cornering(car, v, radius,
                                                delta_guess=delta_guess, vy_guess=vy_guess)
        if not result['converged']:
            break
        _, f_long_achievable, f_drag, f_rr = available_forces(car, v, radius)
        f_long_required = f_drag + f_rr
        if f_long_required > f_long_achievable:
            break
        best_v = v
        best_result = result
        delta_guess, vy_guess = result['delta'], result['vy']
    return best_v, best_result
