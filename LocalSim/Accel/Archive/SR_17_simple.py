import numpy as np

class SR_17_simple:
    """Same car as SR_17, but with the slip-ratio tire model and the torsional
    (half-shaft + wheel/rotor inertia) drivetrain removed - traction is a plain
    mu*Fz cap and the driveline is treated as rigid (motor RPM comes directly
    from vehicle speed). Suspension dynamics (front/rear spring-damper) are
    unchanged from SR_17, since those aren't part of the tire model.
    """
    def __init__(self):
        # Mass & Geometry Parameters
        self.mass = 274.1              # Total mass with driver (kg)
        self.unsprung_mass_corner = 7.76643991  # Wheel/tire/upright/brake mass, per corner (kg)
        self.sprung_mass = self.mass - 4 * self.unsprung_mass_corner  # Chassis + driver, isolated by the springs (kg)
        self.rotational_inertia_factor = 1.05  # Rotational mass correction factor for all four wheels + driveline
                                                # (combined again, since there's no separate driveline inertia state here)
        self.wheelbase = 1.53924          # Wheelbase L (m)
        self.cg_height = 0.32512           # Center of gravity height h_cg (m)
        self.static_rear_bias = 0.509    # 50.9% static weight on rear axle
        self.wheel_radius = 0.203       # Effective tire radius (m)

        # Aerodynamics Parameters
        self.air_density = 1.225        # kg/m^3
        self.frontal_area = 1.16         # Frontal area A (m^2)
        self.cd = 1.52                  # Drag coefficient
        self.cl = 4.08                  # Lift coefficient (downforce)
        self.aero_rear_bias = 0.5116      # Downforce split on rear axle

        # Tire & Rolling Resistance (simple constant peak coefficient, no slip curve)
        self.tire_mu = 1.2              # Peak friction coefficient
        self.crr = 0.015                # Rolling resistance coefficient

        # Suspension Parameters - Front
        self.spring_rate_front = 70050.73   # Spring rate at the spring, front (N/m)
        self.motion_ratio_front = 0.85     # Wheel travel / spring travel, front
        self.damping_ratio_front = 0.76    # Damper coefficient as a fraction of critical damping, front
        self.anti_lift_front = 0.11        # Fraction of front load transfer (loss) reacted through links while accelerating
        self.anti_dive_front = 0.33        # Fraction of front load transfer (gain) reacted through links while decelerating

        # Suspension Parameters - Rear
        self.spring_rate_rear = 80208.09    # Spring rate at the spring, rear (N/m)
        self.motion_ratio_rear = 0.87      # Wheel travel / spring travel, rear
        self.damping_ratio_rear = 0.68     # Damper coefficient as a fraction of critical damping, rear
        self.anti_squat_rear = 0.44     # Fraction of rear load transfer reacted through suspension links
                                         # instead of showing up as extra dynamic tire load (0 = none, 1 = full)
        self.anti_lift_rear = 0.11      # Same idea, but for rear load transfer during deceleration
                                         # (rear unloading under braking/coast/regen instead of squatting)

        # Suspension Parameters - shared
        self.tire_vertical_spring_rate = 97458.0  # Tire carcass vertical spring rate, kt (N/m)
        self.track_width = 1.2065          # Track width, front & rear assumed equal (m)
        self.roll_stiffness_front = 0.45  # Fraction of lateral load transfer reacted by the front axle
                                           # (used by Skidpad.py; not exercised in a straight-line accel run)

        # Drivetrain & Motor Parameters (single-speed RWD electric, EMRAX 228 MV LC, rigid driveline)
        self.reduction_ratio = 3.5      # Fixed single-stage reduction ratio
        self.drivetrain_efficiency = 0.94  # 96% motor efficiency * ~98% single-stage reduction
        self.throttle_request = 1.0      # Driver torque request, as a fraction of the motor's available torque
        self.speed_limit_rpm = 6500       # Motor controller cuts torque above this mechanical speed limit
        self.power_limit = 80000.0        # Electrical power limit from the inverter/accumulator (W), independent of the torque curve

        # Motor Torque Curve: (RPM, Torque in Nm) - EMRAX 228 Medium Voltage, liquid cooled
        # Flat peak torque (220 Nm) up to the 124 kW peak-power point at 5500 RPM, then constant-power
        # roll-off to the 6500 RPM mechanical speed limit, where the controller cuts torque to zero.
        self.rpm_points = np.array([0, 5500, 6500, 6500.001])
        self.torque_points = np.array([220.0, 220.0, 182.2, 0.0])

    def get_motor_torque(self, rpm):
        """Linearly interpolates motor torque from the RPM curve (0 past the speed limit)."""
        return np.interp(rpm, self.rpm_points, self.torque_points, right=0.0)
