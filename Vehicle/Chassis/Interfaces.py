"""Interfaces for chassis-coupled vertical dynamics."""

from typing import Protocol, Sequence


class SuspensionModel(Protocol):
    """Produces one normal load for each wheel at the plant timestep."""

    def step(self, dt_s: float) -> Sequence[float]:
        ...
