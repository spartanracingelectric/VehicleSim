import numpy as np
import matplotlib.pyplot as plt
from SR_17 import SR_17

def simulate_75m_accel(car, target_distance=75.0, dt=0.0005):
    """Simulates acceleration over the given distance using Euler integration.

    This is a coupled dynamic model, not a quasi-static one:
      - Each axle's normal load comes from a real 2nd-order spring/damper (the actual
        front/rear spring rate, motion ratio, and tire vertical spring rate combine into
        an effective corner stiffness), driven by the weight-transfer force, rather than
        being applied to the tire instantaneously.
      - The rear driveline is a two-mass torsional system: motor rotor inertia and rear
        wheel/tire inertia connected by a compliant half-shaft, so wheel speed can differ
        from vehicle speed (that's what generates slip).
      - Rear tire force comes from a Dugoff-style combined-slip model (SR_17.tire_forces,
        ported from the team's TireConfig2.py), not a flat traction cap - Fx saturates
        toward the friction circle limit (tire_mu * Fz) as slip ratio rises, rather than
        being capped or dropping off past a peak.
      - A simple proportional slip controller (launch/traction control) throttles the motor
        torque request back down once slip exceeds a target, matching real launch control.
    """
    g = 9.81
    static_front_bias = 1.0 - car.static_rear_bias
    aero_front_bias = 1.0 - car.aero_rear_bias

    # Effective per-axle suspension properties (spring, in series with tire compliance,
    # reacting the sprung mass on that axle)
    k_wheel_front = car.spring_rate_front * car.motion_ratio_front**2
    k_eff_front_axle = 2.0 * (k_wheel_front * car.tire_vertical_spring_rate) / (k_wheel_front + car.tire_vertical_spring_rate)
    m_front_sprung = car.sprung_mass * static_front_bias
    c_front = car.damping_ratio_front * 2.0 * np.sqrt(k_eff_front_axle * m_front_sprung)

    k_wheel_rear = car.spring_rate_rear * car.motion_ratio_rear**2
    k_eff_rear_axle = 2.0 * (k_wheel_rear * car.tire_vertical_spring_rate) / (k_wheel_rear + car.tire_vertical_spring_rate)
    m_rear_sprung = car.sprung_mass * car.static_rear_bias
    c_rear = car.damping_ratio_rear * 2.0 * np.sqrt(k_eff_rear_axle * m_rear_sprung)

    # Rear driveline: motor rotor inertia reflected to the half-shaft's input side
    I_motor_refl = car.rotor_inertia * car.reduction_ratio**2

    # Initial conditions (small initial velocity to account for starting 0.3m back)
    t, x, v, a = 0.0, 0.0, 2.10277778, 0.0
    z_f, zdot_f = 0.0, 0.0
    z_r, zdot_r = 0.0, 0.0
    omega_wheel = v / car.wheel_radius
    omega_motor_side = omega_wheel  # gearbox-output-side speed; no initial half-shaft twist or slip
    theta_twist = 0.0

    log = {'t': [], 'x': [], 'v': [], 'a': [], 'rpm': [], 'slip': [], 'fz_rear': [], 'fz_front': []}

    while x < target_distance:
        # Aero forces
        f_drag = 0.5 * car.air_density * car.cd * car.frontal_area * (v**2)
        f_down = 0.5 * car.air_density * car.cl * car.frontal_area * (v**2)

        # Weight-transfer forcing target for each axle's suspension (reduced by anti-squat/
        # anti-lift/anti-dive, whichever applies given the sign of the current acceleration)
        rear_coeff = car.anti_squat_rear if a >= 0.0 else car.anti_lift_rear
        front_coeff = car.anti_lift_front if a >= 0.0 else car.anti_dive_front
        transfer_base = (car.mass * a * car.cg_height) / car.wheelbase
        f_transfer_rear_target = transfer_base * (1.0 - rear_coeff)
        f_transfer_front_target = -transfer_base * (1.0 - front_coeff)

        # Suspension dynamics: each axle is a real spring/damper responding to that target,
        # not an instantaneous transfer - this is what lets load transfer lag under a sudden
        # torque request instead of appearing all at once
        zdotdot_r = (f_transfer_rear_target - k_eff_rear_axle * z_r - c_rear * zdot_r) / m_rear_sprung
        zdotdot_f = (f_transfer_front_target - k_eff_front_axle * z_f - c_front * zdot_f) / m_front_sprung

        f_z_dynamic_rear = k_eff_rear_axle * z_r + c_rear * zdot_r
        f_z_dynamic_front = k_eff_front_axle * z_f + c_front * zdot_f

        f_z_static_rear = car.mass * g * car.static_rear_bias
        f_z_static_front = car.mass * g * static_front_bias
        f_z_aero_rear = f_down * car.aero_rear_bias
        f_z_aero_front = f_down * aero_front_bias

        f_z_rear = max(0.0, f_z_static_rear + f_z_dynamic_rear + f_z_aero_rear)
        f_z_front = max(0.0, f_z_static_front + f_z_dynamic_front + f_z_aero_front)

        # Tire model: rear wheel speed vs. vehicle speed sets slip ratio, which the Dugoff
        # model turns into an actual force (straight line, so slip angle = 0)
        slip_ratio = car.tire_slip_ratio(omega_wheel, v)
        f_traction, _ = car.tire_forces(f_z_rear, slip_ratio)

        # Motor torque at the actual motor RPM (from the gearbox-output-side driveline state,
        # not directly from vehicle speed, since the half-shaft can twist and the wheel can slip)
        actual_motor_omega = omega_motor_side * car.reduction_ratio
        motor_rpm = actual_motor_omega * 60.0 / (2.0 * np.pi)
        torque_avail = car.get_motor_torque(motor_rpm) * car.throttle_request

        # Electrical power limit (inverter/accumulator), independent of the torque curve
        power_draw = torque_avail * actual_motor_omega
        if power_draw > car.power_limit and actual_motor_omega > 1e-6:
            torque_avail = car.power_limit / actual_motor_omega

        # Slip control (launch/traction control): throttle back the torque request once
        # slip ratio exceeds the target, instead of just letting the tire spin up
        if car.enable_launch_control and slip_ratio > car.target_slip_ratio:
            throttle_factor = max(0.0, 1.0 - car.slip_control_gain * (slip_ratio - car.target_slip_ratio))
            torque_command = torque_avail * throttle_factor
        else:
            torque_command = torque_avail

        torque_to_shaft_node = torque_command * car.reduction_ratio * car.drivetrain_efficiency

        # Two-mass torsional driveline: motor-side inertia and rear wheel inertia connected
        # by a compliant half-shaft
        T_shaft = car.half_shaft_stiffness * theta_twist + car.half_shaft_damping * (omega_motor_side - omega_wheel)
        omega_motor_side_dot = (torque_to_shaft_node - T_shaft) / I_motor_refl
        omega_wheel_dot = (T_shaft - f_traction * car.wheel_radius) / car.wheel_inertia_rear
        theta_twist_dot = omega_motor_side - omega_wheel

        # Rolling Resistance
        f_rr = car.crr * car.mass * g

        # Vehicle longitudinal acceleration (only the undriven front wheels' spin-up is left
        # as a lumped mass correction; the rear driveline's rotational inertia is explicit above)
        m_eff = car.mass * car.rotational_inertia_factor_front
        a = (f_traction - f_drag - f_rr) / m_eff

        # Integration step (semi-implicit Euler: rates updated first, then integrated positions)
        v += a * dt
        x += v * dt
        t += dt

        omega_motor_side += omega_motor_side_dot * dt
        omega_wheel += omega_wheel_dot * dt
        theta_twist += theta_twist_dot * dt

        zdot_r += zdotdot_r * dt
        z_r += zdot_r * dt
        zdot_f += zdotdot_f * dt
        z_f += zdot_f * dt

        # Logging
        log['t'].append(t)
        log['x'].append(x)
        log['v'].append(v * 3.6) # Store speed in km/h
        log['a'].append(a / g)   # Store acceleration in g's
        log['rpm'].append(motor_rpm)
        log['slip'].append(slip_ratio)
        log['fz_rear'].append(f_z_rear)
        log['fz_front'].append(f_z_front)

    return t, log

# Run default simulation
fsae_car = SR_17()
elapsed_time, sim_log = simulate_75m_accel(fsae_car)

print(f"--- FSAE 75m Acceleration Simulation (Single-Motor RWD Electric) ---")
print(f"Elapsed Time: {elapsed_time:.3f} s")
print(f"Trap Speed:   {sim_log['v'][-1]:.2f} km/h")
print(f"Max Accel:    {max(sim_log['a']):.2f} g")
print(f"Max Slip:     {max(sim_log['slip']) * 100:.1f} %")

# Sweep SR_17's reduction_ratio and graph its effect on 75m acceleration time
reduction_ratios = np.round(np.arange(2.0, 4.0 + 1e-9, 0.1), 1)
accel_times_vs_ratio = []
for ratio in reduction_ratios:
    sweep_car = SR_17()
    sweep_car.reduction_ratio = ratio
    sweep_time, _ = simulate_75m_accel(sweep_car)
    accel_times_vs_ratio.append(sweep_time)

plt.figure(figsize=(8, 5))
plt.plot(reduction_ratios, accel_times_vs_ratio, marker='o', markersize=3)
plt.xlabel("Reduction Ratio")
plt.ylabel("75m Elapsed Time (s)")
plt.title("SR_17 75m Acceleration Time vs. Reduction Ratio")
plt.grid(True)
graph_path = "SR_17_accel_reduction_ratio_sweep.png"
plt.savefig(graph_path, dpi=150)
print(f"Saved reduction ratio sweep graph to {graph_path}")
