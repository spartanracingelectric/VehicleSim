import numpy as np
import matplotlib.pyplot as plt
from SR_18 import SR_18

def solve_axle_force(car, fz, fx_request):
    """Converts a demanded (uncapped) axle drive force into the force the tire model
    actually delivers, via SR_17/SR_18's shared Dugoff combined-slip tire model.

    Unlike SR_17's rear axle, SR_18's per-wheel driveline has no characterized rotor
    inertia / half-shaft compliance to integrate a real wheel-speed state from, so slip
    ratio can't be driven dynamically here. Instead this treats each axle as reaching
    slip equilibrium instantly each step: it inverts the model's straight-line (alpha=0)
    closed form for the kappa that would produce fx_request, then calls tire_forces()
    with that kappa so the returned force still comes from the exact same model - it
    saturates smoothly toward tire_mu*fz rather than hard-capping at it.
    """
    if fz <= 0.0 or fx_request <= 0.0:
        return 0.0, 0.0

    D = car.tire_mu * fz
    fx_capped = min(fx_request, 0.999999 * D)
    if fx_capped <= 0.5 * D:
        kappahat = fx_capped / D
    else:
        kappahat = D / (4.0 * (D - fx_capped))

    Cs = car.tire_c_long * fz
    kappa = kappahat / (Cs / D + kappahat)

    fx, _ = car.tire_forces(fz, kappa)
    return fx, kappa

def simulate_75m_accel(car, target_distance=75.0, dt=0.0005):
    """Simulates acceleration over the given distance using Euler integration.

    Traction and drive force are computed per axle since weight transfer under
    acceleration shifts load between the front and rear axles, and each axle
    is driven independently by its own pair of motors, commanded per
    torque_split_front rather than evenly. Tire force comes from the Dugoff-style
    combined-slip model shared with SR_17 (see solve_axle_force above) rather than
    a flat tire_mu*Fz traction cap.
    """
    t, x, v, a = 0.0, 0.0, 2.49722222, 0.0  # Initial conditions (small initial velocity to account for starting 0.3m back)

    motors_per_axle = car.num_motors // 2
    static_front_bias = 1.0 - car.static_rear_bias
    aero_front_bias = 1.0 - car.aero_rear_bias

    log = {'t': [], 'x': [], 'v': [], 'a': [], 'rpm': [], 'slip_rear': [], 'slip_front': []}

    while x < target_distance:
        g = 9.81

        # Aero forces
        f_drag = 0.5 * car.air_density * car.cd * car.frontal_area * (v**2)
        f_down = 0.5 * car.air_density * car.cl * car.frontal_area * (v**2)

        # Dynamic weight transfer (rearward under acceleration), reduced per axle by anti-squat/
        # anti-lift, which react part of the transfer geometrically through the suspension links
        # instead of the springs
        f_z_transfer = (car.mass * a * car.cg_height) / car.wheelbase
        f_z_transfer_rear = f_z_transfer * (1.0 - car.anti_squat_rear)
        f_z_transfer_front = f_z_transfer * (1.0 - car.anti_lift_front)

        f_z_static_rear = car.mass * g * car.static_rear_bias
        f_z_static_front = car.mass * g * static_front_bias
        f_z_aero_rear = f_down * car.aero_rear_bias
        f_z_aero_front = f_down * aero_front_bias

        f_z_rear = max(0.0, f_z_static_rear + f_z_transfer_rear + f_z_aero_rear)
        f_z_front = max(0.0, f_z_static_front - f_z_transfer_front + f_z_aero_front)

        # Motor RPM (single fixed reduction ratio, all four wheels at the same speed;
        # nominal wheel speed = v/R - the quasi-static slip solve below doesn't feed
        # back into this, same approximation SR_17 makes for its undriven front wheels)
        wheel_rad_s = v / car.wheel_radius
        motor_rpm = (wheel_rad_s * 60.0) / (2.0 * np.pi) * car.reduction_ratio

        # Drive force from motor torque curve (motors_per_axle identical motors per axle)
        torque = car.get_motor_torque(motor_rpm)
        wheel_torque = torque * car.reduction_ratio * car.drivetrain_efficiency
        f_drive_axle_max = motors_per_axle * wheel_torque / car.wheel_radius  # what either axle's own motors could
                                                                                # deliver alone, at 100% torque split

        # Split the commanded drive force between axles per torque_split_front, scaled up so the
        # axle with the larger share runs its motors at full available torque (the other axle is
        # derated below its own max to hold the ratio) - motors are identical hardware, so this is
        # a controller torque-vectoring bias, not a per-axle difference in what the motors can do
        split_rear = 1.0 - car.torque_split_front
        drive_scale = f_drive_axle_max / max(car.torque_split_front, split_rear)
        f_drive_front = drive_scale * car.torque_split_front
        f_drive_rear = drive_scale * split_rear

        # Tractive force per axle from the Dugoff tire model, given each axle's own Fz
        f_x_rear, slip_rear = solve_axle_force(car, f_z_rear, f_drive_rear)
        f_x_front, slip_front = solve_axle_force(car, f_z_front, f_drive_front)
        f_x = f_x_rear + f_x_front

        # Rolling Resistance
        f_rr = car.crr * car.mass * g

        # Acceleration calculation (with effective rotational inertia)
        m_eff = car.mass * car.rotational_inertia_factor
        a = (f_x - f_drag - f_rr) / m_eff

        # Integration step
        v += a * dt
        x += v * dt
        t += dt

        # Logging
        log['t'].append(t)
        log['x'].append(x)
        log['v'].append(v * 3.6) # Store speed in km/h
        log['a'].append(a / g)   # Store acceleration in g's
        log['rpm'].append(motor_rpm)
        log['slip_rear'].append(slip_rear)
        log['slip_front'].append(slip_front)

    return t, log

# Run default simulation
sr18_car = SR_18()
elapsed_time, sim_log = simulate_75m_accel(sr18_car)

print(f"--- FSAE 75m Acceleration Simulation (4-Motor AWD Electric) ---")
print(f"Elapsed Time: {elapsed_time:.3f} s")
print(f"Trap Speed:   {sim_log['v'][-1]:.2f} km/h")
print(f"Max Accel:    {max(sim_log['a']):.2f} g")
print(f"Max Slip:     rear {max(sim_log['slip_rear']) * 100:.1f} %, front {max(sim_log['slip_front']) * 100:.1f} %")

# Sweep SR_18's reduction_ratio and graph its effect on 75m acceleration time
reduction_ratios = np.round(np.arange(10.0, 15.0 + 1e-9, 0.1), 1)
accel_times_vs_ratio = []
for ratio in reduction_ratios:
    sweep_car = SR_18()
    sweep_car.reduction_ratio = ratio
    sweep_time, _ = simulate_75m_accel(sweep_car)
    accel_times_vs_ratio.append(sweep_time)

plt.figure(figsize=(8, 5))
plt.plot(reduction_ratios, accel_times_vs_ratio, marker='o', markersize=3)
plt.xlabel("Reduction Ratio")
plt.ylabel("75m Elapsed Time (s)")
plt.title("SR_18 75m Acceleration Time vs. Reduction Ratio")
plt.grid(True)
graph_path = "SR_18_accel_reduction_ratio_sweep.png"
plt.savefig(graph_path, dpi=150)
print(f"Saved reduction ratio sweep graph to {graph_path}")

# Sweep SR_18's torque_split_front and graph its effect on 75m acceleration time
torque_splits = np.round(np.arange(0.01, 0.99 + 1e-9, 0.01), 2)
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
split_graph_path = "SR_18_accel_torque_split_sweep.png"
plt.savefig(split_graph_path, dpi=150)
print(f"Saved torque split sweep graph to {split_graph_path}")
