import math
from copy import deepcopy

from Functions.constants import G_MPS2, AIR_DENSITY_KGPM3
from Vehicle.Chassis.Interfaces import SuspensionModel
from Vehicle.Chassis.SteeringConfig import SteeringConfig

FL, FR, RL, RR = 0, 1, 2, 3
WHEELS = 4
MAX_DRIVE_SLIP_RATIO = 0.99


class ChassisConfig:
    """Four-wheel planar chassis with interchangeable tire and suspension models."""

    def __init__(
        self, drag_area_m2, rolling_resistance_coeff, wheelbase_m, cg_height_m,
        rear_weight_fraction, wheel_inertia_kgm2, min_slip_speed_mps=1.0,
        steering=None, track_front_m=1.2, track_rear_m=1.2,
        roll_stiffness_front_fraction=0.5, suspension_model: SuspensionModel = None,
    ):
        if not 0 <= rear_weight_fraction <= 1:
            raise ValueError(f"rear_weight_fraction must be between 0 and 1, got {rear_weight_fraction}")
        if not 0 <= roll_stiffness_front_fraction <= 1:
            raise ValueError("roll_stiffness_front_fraction must be between 0 and 1")
        for name, value in (
            ("wheelbase_m", wheelbase_m), ("wheel_inertia_kgm2", wheel_inertia_kgm2),
            ("min_slip_speed_mps", min_slip_speed_mps), ("track_front_m", track_front_m),
            ("track_rear_m", track_rear_m),
        ):
            if value <= 0:
                raise ValueError(f"{name} must be > 0, got {value}")

        self.drag_area_m2 = drag_area_m2
        self.rolling_resistance_coeff = rolling_resistance_coeff
        self.wheelbase_m = wheelbase_m
        self.cg_height_m = cg_height_m
        self.rear_weight_fraction = rear_weight_fraction
        self.wheel_inertia_kgm2 = wheel_inertia_kgm2
        self.min_slip_speed_mps = min_slip_speed_mps
        self.track_front_m = track_front_m
        self.track_rear_m = track_rear_m
        self.roll_stiffness_front_fraction = roll_stiffness_front_fraction
        self.wheels_per_axle = 2
        self.steering = steering if steering is not None else SteeringConfig(wheelbase_m, track_front_m)
        self.suspension_model = suspension_model

        self.speed_mps = 0.0
        self.distance_m = 0.0
        self.acceleration_mps2 = 0.0
        self.lateral_acceleration_mps2 = 0.0
        self.traction_limited = False
        self.steering_angle_rad = 0.0
        self.lateral_velocity_mps = 0.0
        self.yaw_rate_radps = 0.0
        self.heading_rad = 0.0
        self.x_m = 0.0
        self.y_m = 0.0

        self.wheel_speed_radps = None
        self.slip_ratio = [0.0] * WHEELS
        self.slip_angle_rad = [0.0] * WHEELS
        self.tire_load_N = [0.0] * WHEELS
        self.tire_force_x_N = [0.0] * WHEELS
        self.tire_force_y_N = [0.0] * WHEELS
        self.wheel_limited = [False] * WHEELS
        self.wheel_steer_rad = [0.0] * WHEELS
        self._wheel_tires = None
        self._wheel_tire_source = None

    def update(
        self, wheel_torque_Nm, brake_torque_Nm, driveline_inertia_kgm2,
        mass_kg, tire, dt_s=None, *legacy, steering_angle_rad=0.0,
    ):
        # Compatibility with the former axle API:
        # update(wheel_torque, front_brake, rear_brake, inertia, mass, tire, dt).
        if legacy:
            if len(legacy) != 1 or not hasattr(dt_s, "radius_m"):
                raise TypeError("invalid chassis update arguments")
            front_brake = brake_torque_Nm
            rear_brake = driveline_inertia_kgm2
            driveline_inertia_kgm2, mass_kg, tire, dt_s = mass_kg, tire, dt_s, legacy[0]
            brake_torque_Nm = [front_brake, front_brake, rear_brake, rear_brake]
        self._validate_timestep(dt_s)
        torques = self.per_wheel(wheel_torque_Nm)
        brakes = self.per_wheel(brake_torque_Nm)
        inertias = self.per_wheel(driveline_inertia_kgm2)
        tires = self.get_wheel_tires(tire)

        if self.wheel_speed_radps is None:
            self.wheel_speed_radps = [self.speed_mps / tire.radius_m] * WHEELS

        weight_N = mass_kg * G_MPS2
        if self.suspension_model is None:
            self.update_tire_loads(weight_N, mass_kg)
        else:
            self.tire_load_N = self._suspension_loads(dt_s)

        positions = self.get_wheel_positions_m()
        self.wheel_steer_rad = self.get_wheel_steer_angles_rad(steering_angle_rad)
        self.steering_angle_rad = steering_angle_rad
        force_x_body_N = force_y_body_N = yaw_moment_Nm = 0.0
        self.traction_limited = False

        for i in range(WHEELS):
            x_m, y_m = positions[i]
            velocity_x = self.speed_mps - self.yaw_rate_radps * y_m
            velocity_y = self.lateral_velocity_mps + self.yaw_rate_radps * x_m
            steer = self.wheel_steer_rad[i]
            wheel_velocity_x = velocity_x * math.cos(steer) + velocity_y * math.sin(steer)
            wheel_velocity_y = -velocity_x * math.sin(steer) + velocity_y * math.cos(steer)
            reference_speed = max(abs(wheel_velocity_x), self.min_slip_speed_mps)
            self.slip_angle_rad[i] = -math.atan2(wheel_velocity_y, reference_speed)
            self.update_wheel(
                i, tires[i], wheel_velocity_x, reference_speed, torques[i], brakes[i],
                self.wheel_inertia_kgm2 + inertias[i], tire.radius_m, dt_s,
            )
            fx, fy = self.tire_force_x_N[i], self.tire_force_y_N[i]
            force_x_body_N += fx * math.cos(steer) - fy * math.sin(steer)
            force_y_body_N += fx * math.sin(steer) + fy * math.cos(steer)
            yaw_moment_Nm += x_m * (fx * math.sin(steer) + fy * math.cos(steer))
            yaw_moment_Nm -= y_m * (fx * math.cos(steer) - fy * math.sin(steer))

        yaw_inertia_kgm2 = mass_kg * (
            (self.rear_weight_fraction * self.wheelbase_m) ** 2
            + ((1 - self.rear_weight_fraction) * self.wheelbase_m) ** 2
        )
        resistance_N = self.get_drag_force_N() + self.get_rolling_resistance_N(weight_N)
        self.acceleration_mps2 = (force_x_body_N - resistance_N) / mass_kg
        self.lateral_acceleration_mps2 = force_y_body_N / mass_kg
        speed_before_mps = self.speed_mps
        self.speed_mps = max(
            0.0,
            self.speed_mps + (self.acceleration_mps2 + self.lateral_velocity_mps * self.yaw_rate_radps) * dt_s,
        )
        self.lateral_velocity_mps += (
            self.lateral_acceleration_mps2 - speed_before_mps * self.yaw_rate_radps
        ) * dt_s
        self.yaw_rate_radps += yaw_moment_Nm / yaw_inertia_kgm2 * dt_s
        self.heading_rad += self.yaw_rate_radps * dt_s
        self.x_m += (self.speed_mps * math.cos(self.heading_rad) - self.lateral_velocity_mps * math.sin(self.heading_rad)) * dt_s
        self.y_m += (self.speed_mps * math.sin(self.heading_rad) + self.lateral_velocity_mps * math.cos(self.heading_rad)) * dt_s
        self.distance_m += self.speed_mps * dt_s

    def update_tire_loads(self, weight_N, mass_kg):
        longitudinal_N = mass_kg * self.acceleration_mps2 * self.cg_height_m / self.wheelbase_m
        lateral_N = mass_kg * self.lateral_acceleration_mps2 * self.cg_height_m
        front_lateral_N = lateral_N * self.roll_stiffness_front_fraction / self.track_front_m
        rear_lateral_N = lateral_N * (1 - self.roll_stiffness_front_fraction) / self.track_rear_m
        front_static_N = weight_N * (1 - self.rear_weight_fraction) / 2
        rear_static_N = weight_N * self.rear_weight_fraction / 2
        self.tire_load_N = [
            max(0.0, front_static_N - longitudinal_N / 2 - front_lateral_N / 2),
            max(0.0, front_static_N - longitudinal_N / 2 + front_lateral_N / 2),
            max(0.0, rear_static_N + longitudinal_N / 2 - rear_lateral_N / 2),
            max(0.0, rear_static_N + longitudinal_N / 2 + rear_lateral_N / 2),
        ]

    def update_wheel(self, i, tire, wheel_velocity_x, reference_speed, drive_torque_Nm,
                     brake_torque_Nm, inertia_kgm2, radius_m, dt_s):
        wheel_speed = self.wheel_speed_radps[i]
        slip = self.get_slip_ratio(wheel_speed, radius_m, wheel_velocity_x, reference_speed)
        force_x, force_y = tire.calculateForces(self.tire_load_N[i], slip, self.slip_angle_rad[i], dt_s=dt_s)
        force_x, force_y = float(force_x), float(force_y)
        self.wheel_speed_radps[i] = max(
            0.0,
            wheel_speed + (drive_torque_Nm - brake_torque_Nm - radius_m * force_x) / inertia_kgm2 * dt_s,
        )
        self.slip_ratio[i] = slip
        self.tire_force_x_N[i] = force_x
        self.tire_force_y_N[i] = force_y
        self.wheel_limited[i] = bool(tire.isAtGripLimit())
        self.traction_limited = self.traction_limited or self.wheel_limited[i]

    def _suspension_loads(self, dt_s):
        loads = list(self.suspension_model.step(dt_s))
        if len(loads) != WHEELS:
            raise ValueError("suspension model must return four loads in FL, FR, RL, RR order")
        if not all(math.isfinite(load) and load >= 0 for load in loads):
            raise ValueError("suspension model returned invalid wheel loads")
        return loads

    def get_wheel_positions_m(self):
        front = self.rear_weight_fraction * self.wheelbase_m
        rear = front - self.wheelbase_m
        return [(front, self.track_front_m / 2), (front, -self.track_front_m / 2),
                (rear, self.track_rear_m / 2), (rear, -self.track_rear_m / 2)]

    def get_wheel_steer_angles_rad(self, steering_angle_rad):
        left, right = self.steering.wheel_angles(steering_angle_rad)
        return [left, right, 0.0, 0.0]

    @staticmethod
    def get_slip_ratio(wheel_speed, radius_m, wheel_velocity_x, reference_speed):
        slip_ratio = (wheel_speed * radius_m - wheel_velocity_x) / reference_speed
        return max(-1.0, min(MAX_DRIVE_SLIP_RATIO, slip_ratio))

    def get_wheel_tires(self, tire):
        if self._wheel_tires is None or tire is not self._wheel_tire_source:
            self._wheel_tires = (
                tire.createWheelModels(WHEELS)
                if hasattr(tire, "createWheelModels")
                else [deepcopy(tire) for _ in range(WHEELS)]
            )
            self._wheel_tire_source = tire
        return self._wheel_tires

    @staticmethod
    def per_wheel(value):
        if isinstance(value, (str, bytes)):
            raise ValueError("wheel values must be numeric")
        try:
            values = [float(v) for v in value]
        except TypeError:
            return [float(value)] * WHEELS
        if len(values) != WHEELS:
            raise ValueError("expected one value or four values in FL, FR, RL, RR order")
        return values

    @staticmethod
    def _validate_timestep(dt_s):
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be a finite value greater than 0")

    def get_drag_force_N(self):
        return 0.5 * AIR_DENSITY_KGPM3 * self.drag_area_m2 * self.speed_mps ** 2

    def get_rolling_resistance_N(self, weight_N):
        return self.rolling_resistance_coeff * weight_N if self.speed_mps > 0 else 0.0

    # Compatibility accessors for existing callers.
    def getSpeed_mps(self):
        return self.speed_mps

    def getDistance_m(self):
        return self.distance_m

    def getWheelSpeed_radps(self, radius_m):
        return sum(self.getWheelSpeeds_radps(radius_m)) / WHEELS

    def getWheelSpeeds_radps(self, radius_m):
        return [self.speed_mps / radius_m] * WHEELS if self.wheel_speed_radps is None else list(self.wheel_speed_radps)

    def getSlipRatios(self):
        return list(self.slip_ratio)

    def getFrontSlipRatio(self):
        return self.front_slip_ratio

    def getRearSlipRatio(self):
        return self.rear_slip_ratio

    def getSlipAngles_rad(self):
        return list(self.slip_angle_rad)

    def getTireLoads_N(self):
        return list(self.tire_load_N)

    def getTireForces_N(self):
        return list(zip(self.tire_force_x_N, self.tire_force_y_N))

    @property
    def front_slip_ratio(self):
        return (self.slip_ratio[FL] + self.slip_ratio[FR]) / 2

    @property
    def rear_slip_ratio(self):
        return (self.slip_ratio[RL] + self.slip_ratio[RR]) / 2

    @property
    def front_wheel_speed_radps(self):
        return None if self.wheel_speed_radps is None else (self.wheel_speed_radps[FL] + self.wheel_speed_radps[FR]) / 2

    @property
    def rear_wheel_speed_radps(self):
        return None if self.wheel_speed_radps is None else (self.wheel_speed_radps[RL] + self.wheel_speed_radps[RR]) / 2

    @property
    def front_tire_load_N(self):
        return (self.tire_load_N[FL] + self.tire_load_N[FR]) / 2

    @property
    def rear_tire_load_N(self):
        return (self.tire_load_N[RL] + self.tire_load_N[RR]) / 2

    @property
    def front_tire_force_N(self):
        return (self.tire_force_x_N[FL] + self.tire_force_x_N[FR]) / 2

    @property
    def rear_tire_force_N(self):
        return (self.tire_force_x_N[RL] + self.tire_force_x_N[RR]) / 2

    def isTractionLimited(self):
        return self.traction_limited
