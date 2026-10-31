"""Steering geometry and yaw-moment helpers."""

import math


class SteeringConfig:
    """Front-wheel Ackermann steering geometry.

    Positive steering is a left turn. Returned angles are positive for a
    left turn and account for the different inner and outer wheel radii.
    """

    def __init__(self, wheelbase_m: float, front_track_m: float, rear_track_m: float = 0.0):
        for name, value in (("wheelbase_m", wheelbase_m), ("front_track_m", front_track_m)):
            if value <= 0:
                raise ValueError(f"{name} must be > 0, got {value}")
        if rear_track_m < 0:
            raise ValueError(f"rear_track_m must be >= 0, got {rear_track_m}")
        self.wheelbase_m = float(wheelbase_m)
        self.front_track_m = float(front_track_m)
        self.rear_track_m = float(rear_track_m)

    def wheel_angles(self, steering_angle_rad: float) -> tuple[float, float]:
        """Return ``(left, right)`` front-wheel steer angles in radians."""
        if steering_angle_rad == 0:
            return 0.0, 0.0
        radius_m = self.wheelbase_m / math.tan(abs(steering_angle_rad))
        half_track_m = self.front_track_m / 2.0
        inner = math.atan(self.wheelbase_m / max(radius_m - half_track_m, 1e-12))
        outer = math.atan(self.wheelbase_m / (radius_m + half_track_m))
        if steering_angle_rad > 0:
            return inner, outer
        return -inner, -outer

    def average_angle(self, steering_angle_rad: float) -> float:
        """Return the equivalent bicycle-model steering angle."""
        left, right = self.wheel_angles(steering_angle_rad)
        return (left + right) / 2.0
