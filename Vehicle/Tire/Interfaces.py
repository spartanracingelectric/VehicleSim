"""Common interface for interchangeable tire force models."""

from typing import Protocol


class TireModel(Protocol):
    radius_m: float

    def calculateForces(self, tireDownForce, slip_ratio, slip_angle, dt_s=None):
        ...

    def isAtGripLimit(self) -> bool:
        ...
