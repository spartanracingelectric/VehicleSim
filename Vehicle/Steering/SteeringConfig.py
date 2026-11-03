import math
class SteeringConfig:
    """Ackermann geometry shared by longitudinal and planar model callers."""

    def __init__(self, wheelbase_m, track_m=None, ackerman_fraction=1.0, *,
                 front_track_m=None, rear_track_m=0.0):
        if track_m is None:
            track_m = front_track_m
        elif front_track_m is not None and track_m != front_track_m:
            raise ValueError("track_m and front_track_m must agree")
        if wheelbase_m <= 0 or track_m is None or track_m <= 0 or rear_track_m < 0:
            raise ValueError("Wheelbase and front track must be positive; rear track cannot be negative")
        if not 0 <= ackerman_fraction <= 1:
            raise ValueError("ackerman_fraction must be between zero and one")
        self.wheelbase_m = wheelbase_m
        self.track_m = track_m
        self.front_track_m = track_m
        self.rear_track_m = rear_track_m
        self.ackerman_fraction = ackerman_fraction

    def getWheelAngles_rad(self, delta_rad):

        if delta_rad == 0:
             return 0.0, 0.0

        R_turn = self.wheelbase_m / (math.tan(abs(delta_rad)))

        # inside and outside are how far each wheel is turned from pointing straight ahead
        inside = math.atan(self.wheelbase_m / (R_turn - self.track_m/2))
        outside = math.atan(self.wheelbase_m / (R_turn + self.track_m/2))

        inside_blended = abs(delta_rad) + self.ackerman_fraction * (inside - abs(delta_rad))
        outside_blended = abs(delta_rad) + self.ackerman_fraction * (outside - abs(delta_rad))

        # left turn
        if delta_rad>0:
             return inside_blended, outside_blended

        else:
            return -outside_blended, -inside_blended

    def wheel_angles(self, steering_angle_rad):
        return self.getWheelAngles_rad(steering_angle_rad)

    def average_angle(self, steering_angle_rad):
        left, right = self.getWheelAngles_rad(steering_angle_rad)
        return (left + right) / 2
