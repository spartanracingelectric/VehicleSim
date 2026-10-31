"""Lightweight yaw-moment torque allocator for independent wheel motors."""


class TorqueVectoringConfig:
    """Allocate a total axle torque and requested yaw moment left-to-right.

    The allocator is deliberately independent of the motor model. It becomes
    active when the vehicle has independent wheel/axle motor states; the
    current single-motor VehicleConfig cannot apply four different requests.
    """

    def __init__(self, track_m: float, wheel_radius_m: float, max_delta_torque_Nm: float | None = None):
        if track_m <= 0 or wheel_radius_m <= 0:
            raise ValueError("track_m and wheel_radius_m must be > 0")
        if max_delta_torque_Nm is not None and max_delta_torque_Nm < 0:
            raise ValueError("max_delta_torque_Nm must be >= 0")
        self.track_m = float(track_m)
        self.wheel_radius_m = float(wheel_radius_m)
        self.max_delta_torque_Nm = max_delta_torque_Nm

    def allocate(self, total_torque_Nm: float, yaw_moment_Nm: float) -> tuple[float, float]:
        """Return ``(left_torque_Nm, right_torque_Nm)``.

        A positive yaw moment requests more longitudinal force on the right
        wheel, producing a positive (left-turn) yaw moment.
        """
        delta_torque = 2.0 * yaw_moment_Nm * self.wheel_radius_m / self.track_m
        if self.max_delta_torque_Nm is not None:
            delta_torque = max(-self.max_delta_torque_Nm, min(self.max_delta_torque_Nm, delta_torque))
        left = total_torque_Nm / 2.0 - delta_torque / 2.0
        right = total_torque_Nm / 2.0 + delta_torque / 2.0
        return left, right
