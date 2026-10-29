import math
class SkidPadConfig:
    def __init__(self, mass, rear_weight_distribution, cg_height, wheelbase, front_track, rear_track, front_lateral_transfer, yaw_inertia, wheel_radius, wheel_inertia, rotor_inertia, gear_ratio, rolling_resistance, min_slip_speed, tire, aero, steering, limits):
        self.mass = mass
        self.rear_weight_distribution = rear_weight_distribution
        self.cg_height = cg_height
        self.wheelbase = wheelbase
        self.front_track = front_track
        self.rear_track = rear_track
        self.front_lateral_transfer = front_lateral_transfer
        self.yaw_inertia = yaw_inertia
        self.wheel_radius = wheel_radius
        self.wheel_inertia = wheel_inertia
        self.rotor_inertia = rotor_inertia
        self.gear_ratio = gear_ratio
        self.rolling_resistance = rolling_resistance
        self.min_slip_speed = min_slip_speed
        self.tire = tire
        self.aero = aero
        self.steering = steering
        self.limits = limits

        if mass<=0 or cg_height<=0 or wheelbase<=0 or front_track<=0 or rear_track<=0 or front_lateral_transfer<=0 or yaw_inertia<=0 or wheel_radius<=0 or wheel_inertia<=0 or rotor_inertia<=0 or gear_ratio<=0 or min_slip_speed<=0:
            raise ValueError("Invalid SkidPadConfig parameters")

        if  rear_weight_distribution>1 or rear_weight_distribution<0 or front_lateral_transfer>1 or front_lateral_transfer<0:
            raise ValueError("Invalid SkidPadConfig parameters")

        if rolling_resistance<0:
            raise ValueError("Invalid SkidPadConfig parameters")

        if tire is not None and (tire.radius_m<wheel_radius-.01 or tire.radius_m>wheel_radius+.01): #Put some shi here later for steering equaling tracks
            raise ValueError("Invalid SkidPadConfig parameters")

    def getCgToFrontAxle(self) -> float:
        return self.wheelbase * self.rear_weight_distribution

    def getCgToRearAxle(self) -> float:
        return self.wheelbase *  (1 - self.rear_weight_distribution)

    def getTrack(self, wheelIndex):
        if wheelIndex in [0, 1]:
            return self.front_track
        elif wheelIndex in [2, 3]:
            return self.rear_track
        else:
            raise ValueError("Invalid wheel index")

    def getWheelPostions(self, wheelIndex):
        if wheelIndex==0:
            return (self.getCgToFrontAxle(), self.front_track / 2)
        if wheelIndex==1:
            return (self.getCgToFrontAxle(), -(self.front_track / 2))
        if wheelIndex==2:
            return (-self.getCgToRearAxle(), self.rear_track / 2)
        if wheelIndex==3:
            return (-self.getCgToRearAxle(), -(self.rear_track / 2))
        else:
            raise ValueError("Invalid wheel index")

    def getWeight(self):
        return 9.81*self.mass

    def getStaticTireLoadAll(self):
        return [self.getStaticTireLoadsPerTire(i) for i in range(4)]

    def getStaticTireLoadsPerTire(self, wheelIndex):
        if wheelIndex in [0, 1]:
            return (self.getWeight() *  (1 - self.rear_weight_distribution)) / 2
        elif wheelIndex in [2, 3]:
            return (self.getWeight() * self.rear_weight_distribution) / 2
        else:
            raise ValueError("Invalid wheel index")

    def getWheelSpinInertia(self):
        return self.wheel_inertia + (self.rotor_inertia * self.gear_ratio**2)

    def getStartStateOnCircle(self, speed_mps, radius, turn: str = "left") -> tuple:
        if speed_mps <= 0 or radius <= 0:
            raise ValueError("Speed and radius gotta be positive twin.")
        if turn not in ("left", "right"):
            raise ValueError("Turn direction needs to be 'left' or 'right' I think you got a problem")
            
        X = 0.0
        Y = 0.0
        yaw_angle = 0.0
        
        v = 0.0
        
        if turn == "left":
            yaw = speed_mps / radius
            center = (0.0, radius)
        else:
            yaw = -(speed_mps / radius)
            center = (0.0, -radius)
        
        wheel_positions = [self.getWheelPostions(i) for i in range(4)]
        omega_wheels = []
        
        for x, y in wheel_positions:
            v_wheel_ground = math.hypot(speed_mps - yaw*y, yaw*x)
            omega = v_wheel_ground / self.wheel_radius
            omega_wheels.append(omega)
            
        x0 = [
            X, Y, yaw_angle,
            speed_mps, v, yaw,
            omega_wheels[0],
            omega_wheels[1],
            omega_wheels[2],
            omega_wheels[3]
        ]
        
        return x0, center


    def getInsideRearIndex(self, delta_rad):
        if delta_rad > 0:
            return 2
        elif delta_rad < 0:
            return 3

        return None

    def describeStateOnCircle(self, x0, center):
        if len(x0) != 10:
            raise ValueError("Your dum ahh state vector dont got 10 entries")
        return {
            "X": x0[0],
            "Y": x0[1],
            "yaw_angle": x0[2],
            "speed_mps": x0[3],
            "v": x0[4],
            "yaw": x0[5],
            "wheel_r_speed_FL": x0[6],
            "wheel_r_speed_FR": x0[7],
            "wheel_r_speed_RL": x0[8],
            "wheel_r_speed_RR": x0[9],
            "center": center  
        }

    