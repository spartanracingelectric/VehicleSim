# Sweep settings: turn each sweep on/off and set its range as (start, end, step),
# end included. Each sweep re-runs the full sim once per value.
RUN_FINAL_DRIVE_RATIO_SWEEP = True
FINAL_DRIVE_RATIO_RANGE = (2.0, 7.0, 0.1)

RUN_TARGET_SLIP_RATIO_SWEEP = True
TARGET_SLIP_RATIO_RANGE = (0.05, 1.0, 0.01)

import sys
from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent
RESOURCES_DIR = SCRIPT_DIR.parent / "Resources"
OUTPUT_DIR = SCRIPT_DIR.parent / "Outputs" / "Acceleration RWD"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(RESOURCES_DIR / "Car_Setups"))

import numpy as np
import matplotlib.pyplot as plt
from SR_17 import SR_17

def simulate_75m_accel(car, target_distance=75.0, dt=0.0005, locked_diff=True):
    """Simulates a straight-line run over target_distance (m) using semi-implicit
    Euler integration. Returns (elapsed time in s, log dict). locked_diff=True (the
    car's Accel setup) locks the diff; False models an open diff.

    Two-track model driving dead straight (no yaw, no lateral load transfer):
      - Suspension: each axle is a spring/damper (spring rate x motion ratio^2, in
        series with tire vertical stiffness) driven by longitudinal weight transfer,
        so load transfer lags behind torque instead of appearing instantly.
        Anti-squat/anti-lift/anti-dive reduce the transferred load. Each axle's load
        is split evenly left/right.
      - Driveline: one motor (rotor inertia reflected through the final drive ratio)
        drives the diff, which feeds two compliant half-shafts to two independent
        rear wheels, so each rear wheel has its own speed and slip ratio.
      - Locked diff: both half-shafts are rigidly tied to the carrier, so each wheel
        gets whatever torque its own shaft carries (more to the wheel with more grip).
        Open diff: both half-shafts always get equal torque, so one spinning wheel
        cuts torque to both.
      - Tires: car.tire_forces (Pacejka R20 fit to TTC data, scaled by
        TRACK_GRIP_SCALE). Force holds flat at its peak beyond peak slip.
      - Motor: torque curve x throttle_request, capped by car.power_limit.
      - Launch control: proportional torque cut when the worse rear wheel's slip
        exceeds car.target_slip_ratio.
      - Front wheels are undriven; their spin-up inertia is a mass multiplier
        (rotational_inertia_factor_front).
    """
    g = 9.81
    static_front_bias = 1.0 - car.static_rear_bias
    aero_front_bias = 1.0 - car.aero_rear_bias

    # Per-axle suspension stiffness (wheel rate in series with tire, x2 corners) and
    # damping (damping ratio x critical damping on that axle's sprung mass)
    k_wheel_front = car.spring_rate_front * car.motion_ratio_front**2
    k_eff_front_axle = 2.0 * (k_wheel_front * car.tire_vertical_spring_rate) / (k_wheel_front + car.tire_vertical_spring_rate)
    m_front_sprung = car.sprung_mass * static_front_bias
    c_front = car.damping_ratio_front * 2.0 * np.sqrt(k_eff_front_axle * m_front_sprung)

    k_wheel_rear = car.spring_rate_rear * car.motion_ratio_rear**2
    k_eff_rear_axle = 2.0 * (k_wheel_rear * car.tire_vertical_spring_rate) / (k_wheel_rear + car.tire_vertical_spring_rate)
    m_rear_sprung = car.sprung_mass * car.static_rear_bias
    c_rear = car.damping_ratio_rear * 2.0 * np.sqrt(k_eff_rear_axle * m_rear_sprung)

    # Driveline inertias: motor rotor reflected to the diff carrier, and each rear
    # wheel (wheel_inertia_rear is both sides combined)
    I_motor_refl = car.rotor_inertia * car.final_drive_ratio**2
    I_wheel_each = car.wheel_inertia_rear / 2.0

    # Initial conditions: starting speed approximates the 0.3 m rollout before the
    # timing line; no initial suspension travel, half-shaft twist, or wheel slip
    t, x, v, a = 0.0, 0.0, 2.33333, 0.0
    z_f, zdot_f = 0.0, 0.0
    z_r, zdot_r = 0.0, 0.0
    omega_wheel_rl = v / car.wheel_radius
    omega_wheel_rr = v / car.wheel_radius
    omega_motor_side = omega_wheel_rl  # diff carrier speed (wheel side of the final drive)
    theta_twist_rl = 0.0
    theta_twist_rr = 0.0

    log = {'t': [], 'x': [], 'v': [], 'a': [], 'rpm': [], 'slip_rl': [], 'slip_rr': [],
           'fz_rear': [], 'fz_front': []}

    while x < target_distance:
        # Aero
        f_drag = 0.5 * car.air_density * car.cd * car.frontal_area * (v**2)
        f_down = 0.5 * car.air_density * car.cl * car.frontal_area * (v**2)

        # Weight-transfer target per axle, reduced by anti-geometry (anti-squat/anti-lift
        # when accelerating, anti-lift/anti-dive when decelerating)
        rear_coeff = car.anti_squat_rear if a >= 0.0 else car.anti_lift_rear
        front_coeff = car.anti_lift_front if a >= 0.0 else car.anti_dive_front
        transfer_base = (car.mass * a * car.cg_height) / car.wheelbase
        f_transfer_rear_target = transfer_base * (1.0 - rear_coeff)
        f_transfer_front_target = -transfer_base * (1.0 - front_coeff)

        # Suspension spring/damper response to that target
        zdotdot_r = (f_transfer_rear_target - k_eff_rear_axle * z_r - c_rear * zdot_r) / m_rear_sprung
        zdotdot_f = (f_transfer_front_target - k_eff_front_axle * z_f - c_front * zdot_f) / m_front_sprung

        f_z_dynamic_rear = k_eff_rear_axle * z_r + c_rear * zdot_r
        f_z_dynamic_front = k_eff_front_axle * z_f + c_front * zdot_f

        # Axle normal loads = static + dynamic transfer + aero downforce, split evenly
        # left/right on the rear
        f_z_static_rear = car.mass * g * car.static_rear_bias
        f_z_static_front = car.mass * g * static_front_bias
        f_z_aero_rear = f_down * car.aero_rear_bias
        f_z_aero_front = f_down * aero_front_bias

        f_z_rear = max(0.0, f_z_static_rear + f_z_dynamic_rear + f_z_aero_rear)
        f_z_front = max(0.0, f_z_static_front + f_z_dynamic_front + f_z_aero_front)
        f_z_rl = f_z_rear / 2.0
        f_z_rr = f_z_rear / 2.0

        # Rear tire forces from each wheel's own slip ratio and load (slip angle = 0)
        slip_rl = car.tire_slip_ratio(omega_wheel_rl, v)
        slip_rr = car.tire_slip_ratio(omega_wheel_rr, v)
        f_traction_rl, _ = car.tire_forces(f_z_rl, slip_rl)
        f_traction_rr, _ = car.tire_forces(f_z_rr, slip_rr)
        f_traction = f_traction_rl + f_traction_rr

        # Motor torque at the actual motor speed (from the diff carrier, not vehicle speed)
        actual_motor_omega = omega_motor_side * car.final_drive_ratio
        motor_rpm = actual_motor_omega * 60.0 / (2.0 * np.pi)
        torque_avail = car.get_motor_torque(motor_rpm) * car.throttle_request

        # Power limit: caps torque x speed at car.power_limit. drivetrain_efficiency (motor +
        # gearbox losses) is applied after, so this acts as a battery-side power cap
        # (inverter losses not modeled)
        power_draw = torque_avail * actual_motor_omega
        if power_draw > car.power_limit and actual_motor_omega > 1e-6:
            torque_avail = car.power_limit / actual_motor_omega

        # Launch control: one motor, so it reacts to whichever rear wheel slips more
        slip_worst = max(slip_rl, slip_rr)
        if car.enable_launch_control and slip_worst > car.target_slip_ratio:
            throttle_factor = max(0.0, 1.0 - car.slip_control_gain * (slip_worst - car.target_slip_ratio))
            torque_command = torque_avail * throttle_factor
        else:
            torque_command = torque_avail

        torque_to_diff = torque_command * car.final_drive_ratio * car.drivetrain_efficiency

        # Half-shaft torques: each shaft's spring/damper torque from its twist and its
        # speed relative to the carrier
        T_raw_rl = car.half_shaft_stiffness * theta_twist_rl + car.half_shaft_damping * (omega_motor_side - omega_wheel_rl)
        T_raw_rr = car.half_shaft_stiffness * theta_twist_rr + car.half_shaft_damping * (omega_motor_side - omega_wheel_rr)
        if locked_diff:
            # Locked: each wheel gets its own shaft's torque
            T_wheel_rl, T_wheel_rr = T_raw_rl, T_raw_rr
        else:
            # Open: both wheels get the average (equal-torque split)
            T_wheel_rl = T_wheel_rr = 0.5 * (T_raw_rl + T_raw_rr)

        omega_motor_side_dot = (torque_to_diff - (T_raw_rl + T_raw_rr)) / I_motor_refl
        omega_wheel_rl_dot = (T_wheel_rl - f_traction_rl * car.wheel_radius) / I_wheel_each
        omega_wheel_rr_dot = (T_wheel_rr - f_traction_rr * car.wheel_radius) / I_wheel_each
        theta_twist_rl_dot = omega_motor_side - omega_wheel_rl
        theta_twist_rr_dot = omega_motor_side - omega_wheel_rr

        # Rolling resistance on total tire load (weight + downforce)
        f_rr = car.crr * (car.mass * g + f_down)

        # Vehicle acceleration (front wheel inertia as a mass multiplier; rear driveline
        # inertia is already handled above)
        m_eff = car.mass * car.rotational_inertia_factor_front
        a = (f_traction - f_drag - f_rr) / m_eff

        # Semi-implicit Euler: update rates first, then positions
        v += a * dt
        x += v * dt
        t += dt

        omega_motor_side += omega_motor_side_dot * dt
        omega_wheel_rl += omega_wheel_rl_dot * dt
        omega_wheel_rr += omega_wheel_rr_dot * dt
        theta_twist_rl += theta_twist_rl_dot * dt
        theta_twist_rr += theta_twist_rr_dot * dt

        zdot_r += zdotdot_r * dt
        z_r += zdot_r * dt
        zdot_f += zdotdot_f * dt
        z_f += zdot_f * dt

        log['t'].append(t)
        log['x'].append(x)
        log['v'].append(v * 3.6) # km/h
        log['a'].append(a / g)   # g
        log['rpm'].append(motor_rpm)
        log['slip_rl'].append(slip_rl)
        log['slip_rr'].append(slip_rr)
        log['fz_rear'].append(f_z_rear)
        log['fz_front'].append(f_z_front)

    return t, log

# Baseline run
fsae_car = SR_17()
elapsed_time, sim_log = simulate_75m_accel(fsae_car)

print(f"--- FSAE 75m Acceleration Simulation (Single-Motor RWD Electric, Locked Diff) ---")
print(f"Elapsed Time: {elapsed_time:.3f} s")
print(f"Trap Speed:   {sim_log['v'][-1]:.2f} km/h")
print(f"Max Accel:    {max(sim_log['a']):.2f} g")
print(f"Max Slip:     RL {max(sim_log['slip_rl']) * 100:.1f} %, RR {max(sim_log['slip_rr']) * 100:.1f} %")

# Baseline slip ratio vs. distance for both rear wheels (they should overlap, since
# straight-line loads are identical left/right)
plt.figure(figsize=(8, 5))
plt.plot(sim_log['x'], np.array(sim_log['slip_rl']) * 100.0, label="Rear Left")
plt.plot(sim_log['x'], np.array(sim_log['slip_rr']) * 100.0, label="Rear Right", linestyle='--')
plt.xlabel("Distance [m]")
plt.ylabel("Slip Ratio [%]")
plt.title("SR_17 75m Acceleration Slip Ratio vs. Distance")
plt.legend()
plt.grid(True)
slip_trace_path = OUTPUT_DIR / "SR_17_accel_slip_trace.png"
plt.savefig(slip_trace_path, dpi=150)
print(f"Saved slip ratio trace graph to {slip_trace_path}")

def sweep_values(start, end, step):
    """Values from start to end (inclusive) in steps of step, rounded to clear float noise."""
    return np.round(np.arange(start, end + step / 1000.0, step), 6)

# Sweep: 75m time vs. final drive ratio
if RUN_FINAL_DRIVE_RATIO_SWEEP:
    final_drive_ratios = sweep_values(*FINAL_DRIVE_RATIO_RANGE)
    accel_times_vs_ratio = []
    for ratio in final_drive_ratios:
        sweep_car = SR_17()
        sweep_car.final_drive_ratio = ratio
        sweep_time, _ = simulate_75m_accel(sweep_car)
        accel_times_vs_ratio.append(sweep_time)

    plt.figure(figsize=(8, 5))
    plt.plot(final_drive_ratios, accel_times_vs_ratio, marker='o', markersize=3)
    plt.xlabel("Final Drive Ratio")
    plt.ylabel("75m Elapsed Time (s)")
    plt.title("SR_17 75m Acceleration Time vs. Final Drive Ratio")
    plt.grid(True)
    graph_path = OUTPUT_DIR / "SR_17_accel_final_drive_ratio_sweep.png"
    plt.savefig(graph_path, dpi=150)
    print(f"Saved final drive ratio sweep graph to {graph_path}")

# Sweep: 75m time vs. launch control target slip ratio (launch control forced on).
# Time goes flat once the target is above the highest slip the run reaches, since
# launch control then never acts.
if RUN_TARGET_SLIP_RATIO_SWEEP:
    target_slip_ratios = sweep_values(*TARGET_SLIP_RATIO_RANGE)
    accel_times_vs_slip = []
    for tsr in target_slip_ratios:
        sweep_car = SR_17()
        sweep_car.enable_launch_control = True
        sweep_car.target_slip_ratio = tsr
        sweep_time, _ = simulate_75m_accel(sweep_car)
        accel_times_vs_slip.append(sweep_time)

    plt.figure(figsize=(8, 5))
    plt.plot(target_slip_ratios, accel_times_vs_slip, marker='o', markersize=3)
    plt.xlabel("Target Slip Ratio")
    plt.ylabel("75m Elapsed Time (s)")
    plt.title("SR_17 75m Acceleration Time vs. Target Slip Ratio")
    plt.grid(True)
    slip_graph_path = OUTPUT_DIR / "SR_17_accel_target_slip_ratio_sweep.png"
    plt.savefig(slip_graph_path, dpi=150)
    print(f"Saved target slip ratio sweep graph to {slip_graph_path}")
