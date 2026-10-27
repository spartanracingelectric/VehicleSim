# Sweep settings: turn each sweep on/off and set its range as (start, end, step),
# end included. Each sweep re-runs the full sim once per value.
RUN_FINAL_DRIVE_RATIO_SWEEP = True
FINAL_DRIVE_RATIO_RANGE = (10.0, 15.0, 0.1)

RUN_TORQUE_SPLIT_SWEEP = True
TORQUE_SPLIT_RANGE = (0.01, 0.99, 0.01)        # fraction of torque to the front axle

RUN_TARGET_SLIP_RATIO_SWEEP = True
TARGET_SLIP_RATIO_RANGE = (0.05, 1.0, 0.01)

import sys
from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent
RESOURCES_DIR = SCRIPT_DIR.parent / "Resources"
OUTPUT_DIR = SCRIPT_DIR.parent / "Outputs" / "Acceleration AWD"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(RESOURCES_DIR / "Car_Setups"))

import numpy as np
import matplotlib.pyplot as plt
from SR_18 import SR_18

def simulate_75m_accel(car, target_distance=75.0, dt=0.0005):
    """Simulates a straight-line run over target_distance (m) using semi-implicit
    Euler integration. Returns (elapsed time in s, log dict).

    Two-track model driving dead straight (no yaw, no lateral load transfer), same
    structure as Acceleration_RWD.py but with one motor per wheel and no diff:
      - Suspension: each axle is a spring/damper (spring rate x motion ratio^2, in
        series with tire vertical stiffness) driven by longitudinal weight transfer,
        so load transfer lags behind torque instead of appearing instantly.
        Anti-squat/anti-lift/anti-dive reduce the transferred load. Each axle's load
        is split evenly left/right.
      - Driveline: each wheel is its own motor (rotor inertia reflected through the
        final drive ratio) and compliant half-shaft, independent of the other three,
        so each wheel has its own speed and slip ratio.
      - Tires: car.tire_forces (Pacejka R20 fit to TTC data, scaled by
        TRACK_GRIP_SCALE). Force holds flat at its peak beyond peak slip.
      - Motor: torque curve x throttle_request x torque split, with all 4 motors
        sharing car.power_limit.
      - Torque split: the axle with the larger torque_split_front share gets full
        torque; the other is scaled down to hold the ratio. Left/right always equal.
      - Launch control: each motor gets its own proportional torque cut when its
        wheel's slip exceeds car.target_slip_ratio (one motor per wheel, so no need to
        react to the worst wheel like the RWD car does).
      - All wheel inertias are modeled explicitly, so no rotational mass multiplier.
    """
    g = 9.81
    static_front_bias = 1.0 - car.static_rear_bias
    aero_front_bias = 1.0 - car.aero_rear_bias
    motors_per_wheel = car.num_motors // 4

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

    # Driveline inertias per wheel: its motor's rotor reflected through the final drive,
    # and its own wheel (wheel_inertia_corner is per wheel)
    I_motor_wheel_refl = motors_per_wheel * car.rotor_inertia * car.final_drive_ratio**2
    I_wheel_each = car.wheel_inertia_corner

    def launch_control_factor(slip):
        """Fraction of torque a motor keeps: proportional cut once its wheel's slip
        exceeds car.target_slip_ratio (1.0 when launch control is off)."""
        if car.enable_launch_control and slip > car.target_slip_ratio:
            return max(0.0, 1.0 - car.slip_control_gain * (slip - car.target_slip_ratio))
        return 1.0

    # Initial conditions: starting speed approximates the 0.3 m rollout before the
    # timing line; no initial suspension travel, half-shaft twist, or wheel slip
    t, x, v, a = 0.0, 0.0, 2.49722222, 0.0
    z_f, zdot_f = 0.0, 0.0
    z_r, zdot_r = 0.0, 0.0
    omega_wheel_fl = v / car.wheel_radius
    omega_wheel_fr = v / car.wheel_radius
    omega_wheel_rl = v / car.wheel_radius
    omega_wheel_rr = v / car.wheel_radius
    omega_motor_fl = omega_wheel_fl  # motor-side speed (wheel side of the final drive)
    omega_motor_fr = omega_wheel_fr
    omega_motor_rl = omega_wheel_rl
    omega_motor_rr = omega_wheel_rr
    theta_twist_fl = 0.0
    theta_twist_fr = 0.0
    theta_twist_rl = 0.0
    theta_twist_rr = 0.0

    log = {'t': [], 'x': [], 'v': [], 'a': [], 'rpm_front': [], 'rpm_rear': [],
           'slip_front': [], 'slip_rear': [], 'fz_front': [], 'fz_rear': []}

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
        # left/right
        f_z_static_rear = car.mass * g * car.static_rear_bias
        f_z_static_front = car.mass * g * static_front_bias
        f_z_aero_rear = f_down * car.aero_rear_bias
        f_z_aero_front = f_down * aero_front_bias

        f_z_rear = max(0.0, f_z_static_rear + f_z_dynamic_rear + f_z_aero_rear)
        f_z_front = max(0.0, f_z_static_front + f_z_dynamic_front + f_z_aero_front)
        f_z_fl, f_z_fr = f_z_front / 2.0, f_z_front / 2.0
        f_z_rl, f_z_rr = f_z_rear / 2.0, f_z_rear / 2.0

        # Tire forces from each wheel's own slip ratio and load (slip angle = 0)
        slip_fl = car.tire_slip_ratio(omega_wheel_fl, v)
        slip_fr = car.tire_slip_ratio(omega_wheel_fr, v)
        slip_rl = car.tire_slip_ratio(omega_wheel_rl, v)
        slip_rr = car.tire_slip_ratio(omega_wheel_rr, v)
        f_x_fl, _ = car.tire_forces(f_z_fl, slip_fl)
        f_x_fr, _ = car.tire_forces(f_z_fr, slip_fr)
        f_x_rl, _ = car.tire_forces(f_z_rl, slip_rl)
        f_x_rr, _ = car.tire_forces(f_z_rr, slip_rr)
        f_traction = f_x_fl + f_x_fr + f_x_rl + f_x_rr

        # Each motor's actual speed (from its own driveline state, not vehicle speed)
        actual_motor_omega_fl = omega_motor_fl * car.final_drive_ratio
        actual_motor_omega_fr = omega_motor_fr * car.final_drive_ratio
        actual_motor_omega_rl = omega_motor_rl * car.final_drive_ratio
        actual_motor_omega_rr = omega_motor_rr * car.final_drive_ratio
        motor_rpm_fl = actual_motor_omega_fl * 60.0 / (2.0 * np.pi)
        motor_rpm_fr = actual_motor_omega_fr * 60.0 / (2.0 * np.pi)
        motor_rpm_rl = actual_motor_omega_rl * 60.0 / (2.0 * np.pi)
        motor_rpm_rr = actual_motor_omega_rr * 60.0 / (2.0 * np.pi)

        # Front/rear torque split: the larger-share axle gets full torque, the other is
        # scaled to hold the ratio
        split_rear = 1.0 - car.torque_split_front
        norm_max = max(car.torque_split_front, split_rear)
        front_throttle_frac = car.torque_split_front / norm_max
        rear_throttle_frac = split_rear / norm_max

        torque_avail_fl = car.get_motor_torque(motor_rpm_fl) * car.throttle_request * front_throttle_frac
        torque_avail_fr = car.get_motor_torque(motor_rpm_fr) * car.throttle_request * front_throttle_frac
        torque_avail_rl = car.get_motor_torque(motor_rpm_rl) * car.throttle_request * rear_throttle_frac
        torque_avail_rr = car.get_motor_torque(motor_rpm_rr) * car.throttle_request * rear_throttle_frac

        # Launch control, per wheel. Applied before the power limit, so power a slipping
        # wheel can't use is left for the others.
        torque_avail_fl *= launch_control_factor(slip_fl)
        torque_avail_fr *= launch_control_factor(slip_fr)
        torque_avail_rl *= launch_control_factor(slip_rl)
        torque_avail_rr *= launch_control_factor(slip_rr)

        # Power limit: caps total torque x speed across all motors at car.power_limit,
        # scaling every motor down equally. drivetrain_efficiency (motor + gearbox losses)
        # is applied after, so this acts as a battery-side power cap (inverter losses not
        # modeled)
        power_draw_total = motors_per_wheel * (torque_avail_fl * actual_motor_omega_fl
                                                + torque_avail_fr * actual_motor_omega_fr
                                                + torque_avail_rl * actual_motor_omega_rl
                                                + torque_avail_rr * actual_motor_omega_rr)
        if power_draw_total > car.power_limit and power_draw_total > 1e-6:
            power_scale = car.power_limit / power_draw_total
            torque_avail_fl *= power_scale
            torque_avail_fr *= power_scale
            torque_avail_rl *= power_scale
            torque_avail_rr *= power_scale

        torque_to_shaft_fl = motors_per_wheel * torque_avail_fl * car.final_drive_ratio * car.drivetrain_efficiency
        torque_to_shaft_fr = motors_per_wheel * torque_avail_fr * car.final_drive_ratio * car.drivetrain_efficiency
        torque_to_shaft_rl = motors_per_wheel * torque_avail_rl * car.final_drive_ratio * car.drivetrain_efficiency
        torque_to_shaft_rr = motors_per_wheel * torque_avail_rr * car.final_drive_ratio * car.drivetrain_efficiency

        # Per-wheel driveline: motor-side inertia and wheel inertia joined by a compliant
        # half-shaft (spring/damper torque from its twist and speed difference)
        T_shaft_fl = car.half_shaft_stiffness * theta_twist_fl + car.half_shaft_damping * (omega_motor_fl - omega_wheel_fl)
        omega_motor_fl_dot = (torque_to_shaft_fl - T_shaft_fl) / I_motor_wheel_refl
        omega_wheel_fl_dot = (T_shaft_fl - f_x_fl * car.wheel_radius) / I_wheel_each
        theta_twist_fl_dot = omega_motor_fl - omega_wheel_fl

        T_shaft_fr = car.half_shaft_stiffness * theta_twist_fr + car.half_shaft_damping * (omega_motor_fr - omega_wheel_fr)
        omega_motor_fr_dot = (torque_to_shaft_fr - T_shaft_fr) / I_motor_wheel_refl
        omega_wheel_fr_dot = (T_shaft_fr - f_x_fr * car.wheel_radius) / I_wheel_each
        theta_twist_fr_dot = omega_motor_fr - omega_wheel_fr

        T_shaft_rl = car.half_shaft_stiffness * theta_twist_rl + car.half_shaft_damping * (omega_motor_rl - omega_wheel_rl)
        omega_motor_rl_dot = (torque_to_shaft_rl - T_shaft_rl) / I_motor_wheel_refl
        omega_wheel_rl_dot = (T_shaft_rl - f_x_rl * car.wheel_radius) / I_wheel_each
        theta_twist_rl_dot = omega_motor_rl - omega_wheel_rl

        T_shaft_rr = car.half_shaft_stiffness * theta_twist_rr + car.half_shaft_damping * (omega_motor_rr - omega_wheel_rr)
        omega_motor_rr_dot = (torque_to_shaft_rr - T_shaft_rr) / I_motor_wheel_refl
        omega_wheel_rr_dot = (T_shaft_rr - f_x_rr * car.wheel_radius) / I_wheel_each
        theta_twist_rr_dot = omega_motor_rr - omega_wheel_rr

        # Rolling resistance on total tire load (weight + downforce)
        f_rr = car.crr * (car.mass * g + f_down)

        # Vehicle acceleration (all wheel inertias handled above, so plain car.mass)
        a = (f_traction - f_drag - f_rr) / car.mass

        # Semi-implicit Euler: update rates first, then positions
        v += a * dt
        x += v * dt
        t += dt

        omega_motor_fl += omega_motor_fl_dot * dt
        omega_wheel_fl += omega_wheel_fl_dot * dt
        theta_twist_fl += theta_twist_fl_dot * dt

        omega_motor_fr += omega_motor_fr_dot * dt
        omega_wheel_fr += omega_wheel_fr_dot * dt
        theta_twist_fr += theta_twist_fr_dot * dt

        omega_motor_rl += omega_motor_rl_dot * dt
        omega_wheel_rl += omega_wheel_rl_dot * dt
        theta_twist_rl += theta_twist_rl_dot * dt

        omega_motor_rr += omega_motor_rr_dot * dt
        omega_wheel_rr += omega_wheel_rr_dot * dt
        theta_twist_rr += theta_twist_rr_dot * dt

        zdot_r += zdotdot_r * dt
        z_r += zdot_r * dt
        zdot_f += zdotdot_f * dt
        z_f += zdot_f * dt

        # Logged per axle (left/right are identical in a straight line)
        log['t'].append(t)
        log['x'].append(x)
        log['v'].append(v * 3.6)  # km/h
        log['a'].append(a / g)    # g
        log['rpm_front'].append(motor_rpm_fl)
        log['rpm_rear'].append(motor_rpm_rl)
        log['slip_front'].append(max(slip_fl, slip_fr))
        log['slip_rear'].append(max(slip_rl, slip_rr))
        log['fz_front'].append(f_z_front)
        log['fz_rear'].append(f_z_rear)

    return t, log

# Baseline run
sr18_car = SR_18()
elapsed_time, sim_log = simulate_75m_accel(sr18_car)

print(f"--- FSAE 75m Acceleration Simulation (4-Motor AWD Electric) ---")
print(f"Elapsed Time: {elapsed_time:.3f} s")
print(f"Trap Speed:   {sim_log['v'][-1]:.2f} km/h")
print(f"Max Accel:    {max(sim_log['a']):.2f} g")
print(f"Max Slip:     rear {max(sim_log['slip_rear']) * 100:.1f} %, front {max(sim_log['slip_front']) * 100:.1f} %")

# Baseline slip ratio vs. distance, front and rear axles
plt.figure(figsize=(8, 5))
plt.plot(sim_log['x'], np.array(sim_log['slip_front']) * 100.0, label="Front")
plt.plot(sim_log['x'], np.array(sim_log['slip_rear']) * 100.0, label="Rear")
plt.xlabel("Distance [m]")
plt.ylabel("Slip Ratio [%]")
plt.title("SR_18 75m Acceleration Slip Ratio vs. Distance")
plt.legend()
plt.grid(True)
slip_trace_path = OUTPUT_DIR / "SR_18_accel_slip_trace.png"
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
        sweep_car = SR_18()
        sweep_car.final_drive_ratio = ratio
        sweep_time, _ = simulate_75m_accel(sweep_car)
        accel_times_vs_ratio.append(sweep_time)

    plt.figure(figsize=(8, 5))
    plt.plot(final_drive_ratios, accel_times_vs_ratio, marker='o', markersize=3)
    plt.xlabel("Final Drive Ratio")
    plt.ylabel("75m Elapsed Time (s)")
    plt.title("SR_18 75m Acceleration Time vs. Final Drive Ratio")
    plt.grid(True)
    graph_path = OUTPUT_DIR / "SR_18_accel_final_drive_ratio_sweep.png"
    plt.savefig(graph_path, dpi=150)
    print(f"Saved final drive ratio sweep graph to {graph_path}")

# Sweep: 75m time vs. front torque split
if RUN_TORQUE_SPLIT_SWEEP:
    torque_splits = sweep_values(*TORQUE_SPLIT_RANGE)
    accel_times_vs_split = []
    for split in torque_splits:
        sweep_car = SR_18()
        sweep_car.torque_split_front = split
        sweep_time, _ = simulate_75m_accel(sweep_car)
        accel_times_vs_split.append(sweep_time)

    plt.figure(figsize=(8, 5))
    plt.plot(torque_splits, accel_times_vs_split, marker='o', markersize=3)
    plt.xlabel("Front Torque Split (fraction)")
    plt.ylabel("75m Elapsed Time (s)")
    plt.title("SR_18 75m Acceleration Time vs. Front Torque Split")
    plt.grid(True)
    split_graph_path = OUTPUT_DIR / "SR_18_accel_torque_split_sweep.png"
    plt.savefig(split_graph_path, dpi=150)
    print(f"Saved torque split sweep graph to {split_graph_path}")

# Sweep: 75m time vs. launch control target slip ratio (launch control forced on).
# Fastest near the tire's peak slip (~0.10); goes flat at high targets, where launch
# control barely acts.
if RUN_TARGET_SLIP_RATIO_SWEEP:
    target_slip_ratios = sweep_values(*TARGET_SLIP_RATIO_RANGE)
    accel_times_vs_slip = []
    for tsr in target_slip_ratios:
        sweep_car = SR_18()
        sweep_car.enable_launch_control = True
        sweep_car.target_slip_ratio = tsr
        sweep_time, _ = simulate_75m_accel(sweep_car)
        accel_times_vs_slip.append(sweep_time)

    plt.figure(figsize=(8, 5))
    plt.plot(target_slip_ratios, accel_times_vs_slip, marker='o', markersize=3)
    plt.xlabel("Target Slip Ratio")
    plt.ylabel("75m Elapsed Time (s)")
    plt.title("SR_18 75m Acceleration Time vs. Target Slip Ratio")
    plt.grid(True)
    slip_graph_path = OUTPUT_DIR / "SR_18_accel_target_slip_ratio_sweep.png"
    plt.savefig(slip_graph_path, dpi=150)
    print(f"Saved target slip ratio sweep graph to {slip_graph_path}")
