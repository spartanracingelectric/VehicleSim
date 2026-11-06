import math
from copy import deepcopy

from Functions.constants import G_MPS2, AIR_DENSITY_KGPM3
from Functions.FirstOrderFilter import FirstOrderFilter
from Vehicle.Chassis.SteeringConfig import SteeringConfig

FL, FR, RL, RR = 0, 1, 2, 3     # wheel order everywhere: front left, front right, rear left, rear right
WHEELS = 4
MAX_SLIP_RATIO = 0.99           # slip ratio is held to +-0.99, never +-1 (1 + k = 0 in dugoff and a locked wheel is 0/0)

class ChassisConfig:
    # four wheels, each on its own: own load, slip ratio, slip angle, wheel speed, torque, brake and tire force.
    # the car is one body (speed, sideways speed, yaw rate) pushed by the sum of the 4 tires, weight moves between the
    # wheels as the car speeds up / slows down / corners (that's what torque vectoring works with)
    # wheel order is [FL, FR, RL, RR] everywhere
    #
    # drag_area_m2: drag coefficient * frontal area (Cd * A), rolling_resistance_coeff: rolling force / weight
    # wheelbase_m, cg_height_m, rear_weight_fraction: for load transfer (rear_weight_fraction = static weight on the rear)
    # wheel_inertia_kgm2: one wheel + tire + hub + rotor spinning (the motor's rotor through the gearbox is added on top)
    # min_slip_speed_mps: slip divides by the wheel's speed, below this speed it divides by this instead (no / 0 parked)
    #   0.1 m/s, the real car is never exactly parked so the floor only matters in the first moment
    # track_front_m, track_rear_m, roll_stiffness_front_fraction: for the side to side load transfer
    #   TODO: placeholders, no chassis numbers for SR18 yet. track is a typical FSAE ~1.2 m and the roll stiffness split
    #   is a 50/50 guess. they're only here so the corners have something to work with, replace with the real car
    # slip_filter_time_constant_s: first order filter on each wheel's slip ratio, 0 = off (default)
    #   only needed if the slip ratio is seen buzzing, then try 0.001 - 0.005 s. more filter = slower tire response
    def __init__(self, drag_area_m2, rolling_resistance_coeff, wheelbase_m, cg_height_m, rear_weight_fraction,
                 wheel_inertia_kgm2, min_slip_speed_mps=0.1, steering=None, track_front_m=1.2, track_rear_m=1.2,
                 roll_stiffness_front_fraction=0.5, slip_filter_time_constant_s=0.0):
        if not 0 <= rear_weight_fraction <= 1:
            raise ValueError(f"rear_weight_fraction must be between 0 and 1, got {rear_weight_fraction}")
        if not 0 <= roll_stiffness_front_fraction <= 1:
            raise ValueError(f"roll_stiffness_front_fraction must be between 0 and 1, got {roll_stiffness_front_fraction}")
        if slip_filter_time_constant_s < 0:
            raise ValueError("slip_filter_time_constant_s must be >= 0")
        for name, value in (("wheelbase_m", wheelbase_m), ("wheel_inertia_kgm2", wheel_inertia_kgm2),
                            ("min_slip_speed_mps", min_slip_speed_mps), ("track_front_m", track_front_m),
                            ("track_rear_m", track_rear_m)):
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
        self.slip_filter_time_constant_s = slip_filter_time_constant_s
        self.wheels_per_axle = 2
        # no steering given: ackermann from the wheelbase and front track
        self.steering = steering if steering is not None else SteeringConfig(wheelbase_m, track_front_m)

        # starts parked
        self.speed_mps = 0.0
        self.distance_m = 0.0
        self.acceleration_mps2 = 0.0        # along the car
        self.lateral_acceleration_mps2 = 0.0
        self.traction_limited = False
        self.steering_angle_rad = 0.0
        self.lateral_velocity_mps = 0.0
        self.yaw_rate_radps = 0.0
        self.heading_rad = 0.0
        self.x_m = 0.0
        self.y_m = 0.0

        # per wheel [FL, FR, RL, RR]
        # wheel speeds start as None: the first update spins them up to match the car (it may not start parked)
        self.wheel_speed_radps = None
        self.slip_ratio = [0.0] * WHEELS
        self.slip_angle_rad = [0.0] * WHEELS
        self.tire_load_N = [0.0] * WHEELS       # Fz
        self.tire_force_x_N = [0.0] * WHEELS    # Fx along the wheel
        self.tire_force_y_N = [0.0] * WHEELS    # Fy across the wheel
        self.wheel_limited = [False] * WHEELS
        self.wheel_steer_rad = [0.0] * WHEELS

        self._slip_filters = [FirstOrderFilter(slip_filter_time_constant_s) for _ in range(WHEELS)]
        self._wheel_tires = None
        self._wheel_tire_source = None
        self._wheel_tire_model = None

    # wheel_torque_Nm: what the motors put into each wheel through the gearbox, one number for all four or [FL, FR, RL, RR]
    # brake_torque_Nm: most each brake can hold, same shapes (Vehicle/Brake/BrakeConfig.py getWheelTorques_Nm)
    # driveline_inertia_kgm2: the motor's rotor seen at the wheel (Jm * gear_ratio^2), one number or four
    # tire: Vehicle/Tire/Tire.py (dugoff or pacejka), every wheel gets its own copy of the model
    #
    # every wheel:  J dw/dt = wheel torque - brake - r * Fx,   slip = (w r - vx) / max(|vx|, floor),   Fx from the tire
    # the car:      m (dvx/dt - vy yaw) = all the Fx - drag - rolling,  m (dvy/dt + vx yaw) = all the Fy,  Iz dyaw/dt = moments
    # traction_limited: a tire is at the top of its force curve (more torque just spins / locks the wheels)
    # the real car's traction control is in the VCU firmware, so nothing here stops the wheelspin
    #
    # no oscillation: the wheel speed is stepped implicitly (see updateWheel), so even at the 0.1 m/s floor, where the
    # slip changes ~10x faster than the 100 us step, it settles instead of ringing. the other loops (load transfer,
    # car speed) are slow next to the step and have gain < 1
    def update(self, wheel_torque_Nm, brake_torque_Nm, driveline_inertia_kgm2, mass_kg, tire, dt_s, *,
               steering_angle_rad=0.0):
        torques_Nm = self.perWheel(wheel_torque_Nm)
        brakes_Nm = self.perWheel(brake_torque_Nm)
        driveline_kgm2 = self.perWheel(driveline_inertia_kgm2)
        tires = self.getWheelTires(tire)
        radius_m = tire.radius_m
        if self.wheel_speed_radps is None:
            self.wheel_speed_radps = [self.speed_mps / radius_m] * WHEELS

        weight_N = mass_kg * G_MPS2
        self.updateTireLoads(weight_N, mass_kg)

        positions = self.getWheelPositions_m()
        self.wheel_steer_rad = self.getWheelSteerAngles_rad(steering_angle_rad)
        self.steering_angle_rad = steering_angle_rad

        force_x_body_N = 0.0
        force_y_body_N = 0.0
        yaw_moment_Nm = 0.0
        limited = False
        for i in range(WHEELS):
            x_m, y_m = positions[i]
            # the patch's own velocity: the car's plus the yaw turning it around the cg
            velocity_x = self.speed_mps - self.yaw_rate_radps * y_m
            velocity_y = self.lateral_velocity_mps + self.yaw_rate_radps * x_m
            steer = self.wheel_steer_rad[i]
            # same velocity along and across the wheel
            wheel_velocity_x = velocity_x * math.cos(steer) + velocity_y * math.sin(steer)
            wheel_velocity_y = -velocity_x * math.sin(steer) + velocity_y * math.cos(steer)
            reference_speed = max(abs(wheel_velocity_x), self.min_slip_speed_mps)
            self.slip_angle_rad[i] = -math.atan2(wheel_velocity_y, reference_speed)

            self.updateWheel(i, tires[i], wheel_velocity_x, reference_speed, torques_Nm[i], brakes_Nm[i],
                             self.wheel_inertia_kgm2 + driveline_kgm2[i], radius_m, dt_s)
            limited = limited or self.wheel_limited[i]

            # wheel direction -> car direction
            fx = self.tire_force_x_N[i]
            fy = self.tire_force_y_N[i]
            body_fx = fx * math.cos(steer) - fy * math.sin(steer)
            body_fy = fx * math.sin(steer) + fy * math.cos(steer)
            force_x_body_N += body_fx
            force_y_body_N += body_fy
            yaw_moment_Nm += x_m * body_fy - y_m * body_fx
        self.traction_limited = limited

        cg_from_front_m = self.rear_weight_fraction * self.wheelbase_m
        cg_from_rear_m = self.wheelbase_m - cg_from_front_m
        yaw_inertia_kgm2 = mass_kg * (cg_from_front_m ** 2 + cg_from_rear_m ** 2)

        resistance_N = self.getDragForce_N() + self.getRollingResistance_N(weight_N)
        self.acceleration_mps2 = (force_x_body_N - resistance_N) / mass_kg
        self.lateral_acceleration_mps2 = force_y_body_N / mass_kg
        speed_before_mps = self.speed_mps
        self.speed_mps = max(0.0, self.speed_mps + (self.acceleration_mps2 + self.lateral_velocity_mps * self.yaw_rate_radps) * dt_s)
        self.lateral_velocity_mps += (self.lateral_acceleration_mps2 - speed_before_mps * self.yaw_rate_radps) * dt_s
        self.yaw_rate_radps += yaw_moment_Nm / yaw_inertia_kgm2 * dt_s
        self.heading_rad += self.yaw_rate_radps * dt_s
        self.x_m += (self.speed_mps * math.cos(self.heading_rad) - self.lateral_velocity_mps * math.sin(self.heading_rad)) * dt_s
        self.y_m += (self.speed_mps * math.sin(self.heading_rad) + self.lateral_velocity_mps * math.cos(self.heading_rad)) * dt_s
        self.distance_m += self.speed_mps * dt_s

    # (x forward, y left) of each wheel from the cg, in wheel order
    def getWheelPositions_m(self):
        front_m = self.rear_weight_fraction * self.wheelbase_m
        rear_m = front_m - self.wheelbase_m
        return [(front_m, self.track_front_m / 2), (front_m, -self.track_front_m / 2),
                (rear_m, self.track_rear_m / 2), (rear_m, -self.track_rear_m / 2)]

    # steering_angle_rad is the car's (bicycle) angle, positive = left. the front wheels get the ackermann angles
    def getWheelSteerAngles_rad(self, steering_angle_rad):
        left, right = self.steering.wheel_angles(steering_angle_rad)
        return [left, right, 0.0, 0.0]

    # weight shifts to the rear speeding up and to the front braking: m a h / wheelbase, and to the outside wheels in a
    # corner: m ay h / track, shared front / rear by roll_stiffness_front_fraction (simple version: no roll centers)
    # uses last step's accelerations (this step's aren't known until the tires have pushed)
    # TODO: a wheel can't pull its load below 0 so the sum can come up a little short when one lifts
    def updateTireLoads(self, weight_N, mass_kg):
        height_m = self.cg_height_m
        longitudinal_N = mass_kg * self.acceleration_mps2 * height_m / self.wheelbase_m
        lateral_N = mass_kg * self.lateral_acceleration_mps2 * height_m
        front_roll = self.roll_stiffness_front_fraction
        front_lateral_N = lateral_N * front_roll / self.track_front_m
        rear_lateral_N = lateral_N * (1 - front_roll) / self.track_rear_m

        front_static_N = weight_N * (1 - self.rear_weight_fraction) / 2
        rear_static_N = weight_N * self.rear_weight_fraction / 2
        # left lifts when the car accelerates to the left (positive lateral acceleration), right takes the load
        self.tire_load_N = [
            max(0.0, front_static_N - longitudinal_N / 2 - front_lateral_N / 2),
            max(0.0, front_static_N - longitudinal_N / 2 + front_lateral_N / 2),
            max(0.0, rear_static_N + longitudinal_N / 2 - rear_lateral_N / 2),
            max(0.0, rear_static_N + longitudinal_N / 2 + rear_lateral_N / 2),
        ]

    # one wheel for one step: slip -> tire force -> wheel speeds up or slows down
    # the brake can only slow the wheel to a stop, never spin it backwards: once stopped it stays locked
    # as long as the brake can hold more than the motor and road are turning it with
    #
    # the wheel is stepped implicitly: the force changes with wheel speed by slope = dFx/dw, which at the slip floor is
    # huge (r Cx / floor), a plain step with that would overshoot and ring. so the step is divided by (1 + dt r slope / J),
    # this is backward euler on the linearized force: always settles, and ~ the plain step when the slope is small.
    # slope is measured with one extra look at the tire (a bit above the wheel speed) and never below 0, past the peak
    # of the curve the wheel really does run away (locking / spinning), that stays
    def updateWheel(self, i, tire, wheel_velocity_x, reference_speed, drive_torque_Nm, brake_torque_Nm, inertia_kgm2,
                    radius_m, dt_s):
        wheel_speed_radps = self.wheel_speed_radps[i]
        load_N = self.tire_load_N[i]
        slip_angle = self.slip_angle_rad[i]

        # a bit more wheel speed: gives ~0.01 of slip ratio, whatever the speed
        probe_radps = 0.01 * reference_speed / radius_m
        probe_force_N = tire.calculateForces(load_N, self.getSlipRatio(wheel_speed_radps + probe_radps, radius_m, wheel_velocity_x, reference_speed),
                                             slip_angle, dt_s=dt_s, apply_filter=False)[0]

        slip_ratio = self.getSlipRatio(wheel_speed_radps, radius_m, wheel_velocity_x, reference_speed)
        if self.slip_filter_time_constant_s:
            slip_ratio = self._slip_filters[i].update(slip_ratio, dt_s)
        # float(): the tire models use numpy, its numbers are slow if they leak into the motor's 5 us steps
        force_x_N, force_y_N = tire.calculateForces(load_N, slip_ratio, slip_angle, dt_s=dt_s)
        force_x_N = float(force_x_N)
        force_y_N = float(force_y_N)
        self.wheel_limited[i] = bool(tire.isAtGripLimit())

        slope = max((float(probe_force_N) - force_x_N) / probe_radps, 0.0) if probe_radps > 0 else 0.0
        net_torque_Nm = drive_torque_Nm - brake_torque_Nm - radius_m * force_x_N
        step_radps = net_torque_Nm / inertia_kgm2 * dt_s / (1 + dt_s * radius_m * slope / inertia_kgm2)
        self.wheel_speed_radps[i] = max(0.0, wheel_speed_radps + step_radps)
        self.slip_ratio[i] = slip_ratio
        self.tire_force_x_N[i] = force_x_N
        self.tire_force_y_N[i] = force_y_N

    # wheel rim speed against the ground speed under the wheel, divided by the speed (not less than the floor)
    # driving > 0, braking < 0, locked = -0.99 (held to +-0.99, see MAX_SLIP_RATIO)
    def getSlipRatio(self, wheel_speed_radps, radius_m, wheel_velocity_x, reference_speed):
        slip_ratio = (wheel_speed_radps * radius_m - wheel_velocity_x) / reference_speed
        return max(-MAX_SLIP_RATIO, min(MAX_SLIP_RATIO, slip_ratio))

    # the tire models remember their last forces (and filters), so every wheel has its own copy
    def getWheelTires(self, tire):
        model = getattr(tire, "model", None)
        if self._wheel_tires is None or tire is not self._wheel_tire_source or model != self._wheel_tire_model:
            if hasattr(tire, "createWheelModels"):
                self._wheel_tires = tire.createWheelModels(WHEELS)
            else:
                self._wheel_tires = [deepcopy(tire) for _ in range(WHEELS)]
            self._wheel_tire_source = tire
            self._wheel_tire_model = model
        return self._wheel_tires

    # one number -> the same for every wheel, or four numbers [FL, FR, RL, RR]
    @staticmethod
    def perWheel(value):
        try:
            values = [float(v) for v in value]
        except TypeError:
            return [float(value)] * WHEELS
        if len(values) != WHEELS:
            raise ValueError(f"expected 1 or 4 numbers in wheel order [FL, FR, RL, RR], got {len(values)}")
        return values

    def getDragForce_N(self):
        return 0.5 * AIR_DENSITY_KGPM3 * self.drag_area_m2 * self.speed_mps ** 2

    # only while rolling, so a parked car doesn't get pushed backwards
    def getRollingResistance_N(self, weight_N):
        return self.rolling_resistance_coeff * weight_N if self.speed_mps > 0 else 0.0

    def getSpeed_mps(self):
        return self.speed_mps

    def getDistance_m(self):
        return self.distance_m

    # average of the four wheels (the car rolling with no slip before the first update)
    def getWheelSpeed_radps(self, radius_m):
        if self.wheel_speed_radps is None:
            return self.speed_mps / radius_m
        return sum(self.wheel_speed_radps) / WHEELS

    # per wheel [FL, FR, RL, RR]
    def getWheelSpeeds_radps(self, radius_m):
        if self.wheel_speed_radps is None:
            return [self.speed_mps / radius_m] * WHEELS
        return list(self.wheel_speed_radps)

    def getSlipRatios(self):
        return list(self.slip_ratio)

    def getSlipAngles_rad(self):
        return list(self.slip_angle_rad)

    def getTireLoads_N(self):
        return list(self.tire_load_N)

    def getTireForces_N(self):
        return list(zip(self.tire_force_x_N, self.tire_force_y_N))

    # axle averages, for plots and the old per axle numbers
    def getFrontSlipRatio(self):
        return (self.slip_ratio[FL] + self.slip_ratio[FR]) / 2

    def getRearSlipRatio(self):
        return (self.slip_ratio[RL] + self.slip_ratio[RR]) / 2

    @property
    def front_slip_ratio(self):
        return self.getFrontSlipRatio()

    @property
    def rear_slip_ratio(self):
        return self.getRearSlipRatio()

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

    def isTractionLimited(self):
        return self.traction_limited
