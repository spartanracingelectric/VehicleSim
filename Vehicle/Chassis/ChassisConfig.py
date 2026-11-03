import math

from Functions.constants import G_MPS2, AIR_DENSITY_KGPM3

class ChassisConfig:
    # straight line: the tires push the car, drag and rolling resistance hold it back.
    # each axle's wheels spin on their own (wheel slip), weight shifts between the axles as the car speeds up/slows down
    # drag_area_m2: drag coefficient * frontal area (Cd * A), rolling_resistance_coeff: rolling force / weight
    # wheelbase_m, cg_height_m, rear_weight_fraction: for load transfer (rear_weight_fraction = static weight on the rear)
    # wheel_inertia_kgm2: one wheel + tire + hub + rotor spinning (the motor's rotor through the gearbox is added on top)
    # min_slip_speed_mps: slip ratio divides by car speed, below this speed it divides by this instead (no / 0 parked)
    def __init__(self, drag_area_m2, rolling_resistance_coeff, wheelbase_m, cg_height_m, rear_weight_fraction,
                 wheel_inertia_kgm2, min_slip_speed_mps=1.0, steering=None):
        if not 0 <= rear_weight_fraction <= 1:
            raise ValueError(f"rear_weight_fraction must be between 0 and 1, got {rear_weight_fraction}")
        for name, value in (("wheelbase_m", wheelbase_m), ("wheel_inertia_kgm2", wheel_inertia_kgm2),
                            ("min_slip_speed_mps", min_slip_speed_mps)):
            if value <= 0:
                raise ValueError(f"{name} must be > 0, got {value}")

        self.drag_area_m2 = drag_area_m2
        self.rolling_resistance_coeff = rolling_resistance_coeff
        self.wheelbase_m = wheelbase_m
        self.cg_height_m = cg_height_m
        self.rear_weight_fraction = rear_weight_fraction
        self.wheel_inertia_kgm2 = wheel_inertia_kgm2
        self.min_slip_speed_mps = min_slip_speed_mps
        self.wheels_per_axle = 2
        self.steering = steering

        # starts parked
        self.speed_mps = 0.0
        self.distance_m = 0.0
        self.acceleration_mps2 = 0.0
        self.traction_limited = False
        self.steering_angle_rad = 0.0
        self.lateral_velocity_mps = 0.0
        self.yaw_rate_radps = 0.0
        self.heading_rad = 0.0
        self.x_m = 0.0
        self.y_m = 0.0

        # per axle, both wheels on an axle do the same thing in a straight line
        # wheel speeds start as None: the first update spins them up to match the car (it may not start parked)
        self.front_wheel_speed_radps = None
        self.rear_wheel_speed_radps = None
        self.front_slip_ratio = 0.0
        self.rear_slip_ratio = 0.0
        self.front_tire_load_N = 0.0    # Fz per tire
        self.rear_tire_load_N = 0.0
        self.front_tire_force_N = 0.0   # Fx per tire
        self.rear_tire_force_N = 0.0

    # wheel_torque_Nm: what the motors put into each wheel through the gearbox
    # front/rear_brake_torque_Nm: most each brake can hold, per wheel (Vehicle/Brake/BrakeConfig.py)
    # driveline_inertia_kgm2: the motor's rotor seen at the wheel (Jm * gear_ratio^2), spins with each wheel
    # tire: any tire model with calculateForces(Fz, slip_ratio, slip_angle) and isAtGripLimit(), dugoff or magic formula
    #
    # every wheel:  J dw/dt = wheel torque - brake - r * Fx,   slip = (w r - v) / v,   Fx from the tire model
    # the car:      m dv/dt = all the Fx - drag - rolling
    # one step per update (100 us) is fine: the slip settles in J v / (r^2 Cx), ~0.3 ms at min_slip_speed_mps on tire1
    # traction_limited: a tire is at the top of its force curve (more torque just spins / locks the wheels)
    # the real car's traction control is in the VCU firmware, so nothing here stops the wheelspin
    def update(self, wheel_torque_Nm, front_brake_torque_Nm, rear_brake_torque_Nm, driveline_inertia_kgm2,
               mass_kg, tire, dt_s, *, steering_angle_rad=0.0):
        if steering_angle_rad:
            return self.updateSteered(wheel_torque_Nm, front_brake_torque_Nm, rear_brake_torque_Nm,
                                      driveline_inertia_kgm2, mass_kg, tire, dt_s, steering_angle_rad)
        radius_m = tire.radius_m
        if self.front_wheel_speed_radps is None:
            self.front_wheel_speed_radps = self.rear_wheel_speed_radps = self.speed_mps / radius_m

        weight_N = mass_kg * G_MPS2
        self.updateTireLoads(weight_N, mass_kg)
        inertia_kgm2 = self.wheel_inertia_kgm2 + driveline_inertia_kgm2

        self.front_wheel_speed_radps, self.front_slip_ratio, self.front_tire_force_N, _, front_limited = self.updateWheel(
            self.front_wheel_speed_radps, self.front_tire_load_N, wheel_torque_Nm, front_brake_torque_Nm,
            inertia_kgm2, tire, dt_s)
        self.rear_wheel_speed_radps, self.rear_slip_ratio, self.rear_tire_force_N, _, rear_limited = self.updateWheel(
            self.rear_wheel_speed_radps, self.rear_tire_load_N, wheel_torque_Nm, rear_brake_torque_Nm,
            inertia_kgm2, tire, dt_s)
        self.traction_limited = front_limited or rear_limited

        tire_force_N = self.wheels_per_axle * (self.front_tire_force_N + self.rear_tire_force_N)
        self.acceleration_mps2 = (tire_force_N - self.getDragForce_N() - self.getRollingResistance_N(weight_N)) / mass_kg
        self.speed_mps = max(0.0, self.speed_mps + self.acceleration_mps2 * dt_s)
        self.distance_m += self.speed_mps * dt_s

    def updateSteered(self, wheel_torque_Nm, front_brake_torque_Nm, rear_brake_torque_Nm,
                      driveline_inertia_kgm2, mass_kg, tire, dt_s, steering_angle_rad):
        """Advance the car with front steering and a single-track lateral model.

        Longitudinal wheel slip remains fully simulated. Lateral forces use
        each axle's slip angle and feed body lateral velocity and yaw rate.
        This is intentionally a first steering model, not a suspension model.
        """
        radius_m = tire.radius_m
        if self.front_wheel_speed_radps is None:
            self.front_wheel_speed_radps = self.rear_wheel_speed_radps = self.speed_mps / radius_m
        weight_N = mass_kg * G_MPS2
        self.updateTireLoads(weight_N, mass_kg)
        inertia_kgm2 = self.wheel_inertia_kgm2 + driveline_inertia_kgm2
        cg_from_front_m = self.rear_weight_fraction * self.wheelbase_m
        cg_from_rear_m = self.wheelbase_m - cg_from_front_m
        speed_for_angle = max(abs(self.speed_mps), self.min_slip_speed_mps)
        front_slip_angle = steering_angle_rad - math.atan2(
            self.lateral_velocity_mps + cg_from_front_m * self.yaw_rate_radps, speed_for_angle)
        rear_slip_angle = -math.atan2(
            self.lateral_velocity_mps - cg_from_rear_m * self.yaw_rate_radps, speed_for_angle)

        self.front_wheel_speed_radps, self.front_slip_ratio, self.front_tire_force_N, front_fy, front_limited = self.updateWheel(
            self.front_wheel_speed_radps, self.front_tire_load_N, wheel_torque_Nm, front_brake_torque_Nm,
            inertia_kgm2, tire, dt_s, front_slip_angle)
        self.rear_wheel_speed_radps, self.rear_slip_ratio, self.rear_tire_force_N, rear_fy, rear_limited = self.updateWheel(
            self.rear_wheel_speed_radps, self.rear_tire_load_N, wheel_torque_Nm, rear_brake_torque_Nm,
            inertia_kgm2, tire, dt_s, rear_slip_angle)
        self.traction_limited = front_limited or rear_limited
        self.steering_angle_rad = steering_angle_rad

        front_force_y_N = self.wheels_per_axle * front_fy * math.cos(steering_angle_rad)
        rear_force_y_N = self.wheels_per_axle * rear_fy
        lateral_force_N = front_force_y_N + rear_force_y_N
        yaw_moment_Nm = cg_from_front_m * front_force_y_N - cg_from_rear_m * rear_force_y_N
        yaw_inertia_kgm2 = mass_kg * (cg_from_front_m ** 2 + cg_from_rear_m ** 2)
        self.lateral_velocity_mps += (lateral_force_N / mass_kg - self.speed_mps * self.yaw_rate_radps) * dt_s
        self.yaw_rate_radps += yaw_moment_Nm / yaw_inertia_kgm2 * dt_s

        tire_force_N = self.wheels_per_axle * (self.front_tire_force_N + self.rear_tire_force_N)
        self.acceleration_mps2 = (tire_force_N - self.getDragForce_N() - self.getRollingResistance_N(weight_N)) / mass_kg
        self.speed_mps = max(0.0, self.speed_mps + self.acceleration_mps2 * dt_s)
        self.heading_rad += self.yaw_rate_radps * dt_s
        self.x_m += (self.speed_mps * math.cos(self.heading_rad) - self.lateral_velocity_mps * math.sin(self.heading_rad)) * dt_s
        self.y_m += (self.speed_mps * math.sin(self.heading_rad) + self.lateral_velocity_mps * math.cos(self.heading_rad)) * dt_s
        self.distance_m += self.speed_mps * dt_s

    # weight shifts to the rear speeding up and to the front braking: m a h / wheelbase,
    # uses last step's acceleration (this step's isn't known until the tires have pushed)
    def updateTireLoads(self, weight_N, mass_kg):
        transfer_N = mass_kg * self.acceleration_mps2 * self.cg_height_m / self.wheelbase_m
        front_axle_N = max(0.0, weight_N * (1 - self.rear_weight_fraction) - transfer_N)
        rear_axle_N = max(0.0, weight_N * self.rear_weight_fraction + transfer_N)
        self.front_tire_load_N = front_axle_N / self.wheels_per_axle
        self.rear_tire_load_N = rear_axle_N / self.wheels_per_axle

    # one wheel for one step: slip -> tire force -> wheel speeds up or slows down
    # the brake can only slow the wheel to a stop, never spin it backwards: once stopped it stays locked
    # as long as the brake can hold more than the motor and road are turning it with
    def updateWheel(self, wheel_speed_radps, tire_load_N, wheel_torque_Nm, brake_torque_Nm, inertia_kgm2, tire, dt_s,
                    slip_angle=0.0):
        radius_m = tire.radius_m
        # below min_slip_speed_mps the force fades out with the speed difference, so the car settles to a stop smoothly
        # (a locked wheel is -1 down to that speed, then the last bit of braking softens, a few cm of stopping distance)
        slip_ratio = (wheel_speed_radps * radius_m - self.speed_mps) / max(self.speed_mps, self.min_slip_speed_mps)
        # float(): the tire models use numpy, its numbers are slow if they leak into the motor's 5 us steps
        tire_force_N, tire_lateral_force_N = tire.calculateForces(tire_load_N, slip_ratio, slip_angle, dt_s=dt_s)
        tire_force_N = float(tire_force_N)
        tire_lateral_force_N = float(tire_lateral_force_N)
        limited = bool(tire.isAtGripLimit())

        free_wheel_speed_radps = wheel_speed_radps + (wheel_torque_Nm - radius_m * tire_force_N) / inertia_kgm2 * dt_s
        wheel_speed_radps = max(0.0, free_wheel_speed_radps - brake_torque_Nm / inertia_kgm2 * dt_s)
        return wheel_speed_radps, slip_ratio, tire_force_N, tire_lateral_force_N, limited

    def getDragForce_N(self):
        return 0.5 * AIR_DENSITY_KGPM3 * self.drag_area_m2 * self.speed_mps ** 2

    # only while rolling, so a parked car doesn't get pushed backwards
    def getRollingResistance_N(self, weight_N):
        return self.rolling_resistance_coeff * weight_N if self.speed_mps > 0 else 0.0

    def getSpeed_mps(self):
        return self.speed_mps

    def getDistance_m(self):
        return self.distance_m

    # average of the front and rear wheels (the car rolling with no slip before the first update)
    def getWheelSpeed_radps(self, radius_m):
        if self.front_wheel_speed_radps is None:
            return self.speed_mps / radius_m
        return (self.front_wheel_speed_radps + self.rear_wheel_speed_radps) / 2

    def getFrontSlipRatio(self):
        return self.front_slip_ratio

    def getRearSlipRatio(self):
        return self.rear_slip_ratio

    def isTractionLimited(self):
        return self.traction_limited
