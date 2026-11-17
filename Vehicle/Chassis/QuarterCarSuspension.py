"""Real-time bridge from the quarter-car design model to the chassis model."""

import numpy as np

from Vehicle.Suspension.model import QuarterCarModel


class QuarterCarSuspension:
    """Advance four independent quarter-car corners with a fixed time step."""

    def __init__(self, cases):
        if len(cases) != 4:
            raise ValueError("Quarter-car suspension requires four cases in FL, FR, RL, RR order")

        self.models = [QuarterCarModel(case) for case in cases]
        self.states = [np.zeros(4) for _ in cases]
        self.time_s = 0.0
        self.contact_forces_N = [model.static_tire_load for model in self.models]

    def step(self, dt_s):
        if dt_s <= 0:
            raise ValueError("suspension timestep must be greater than 0")

        for index, model in enumerate(self.models):
            state = self.states[index]
            t = self.time_s

            def derivatives(time, values):
                return np.asarray(model.quarter_car(time, values), dtype=float)

            k1 = derivatives(t, state)
            k2 = derivatives(t + dt_s / 2, state + dt_s * k1 / 2)
            k3 = derivatives(t + dt_s / 2, state + dt_s * k2 / 2)
            k4 = derivatives(t + dt_s, state + dt_s * k3)
            state = state + dt_s * (k1 + 2 * k2 + 2 * k3 + k4) / 6

            self.states[index] = state
            _, _, _, _, contact_force, _ = model.forces(
                t + dt_s, state[0], state[1], state[2], state[3]
            )
            self.contact_forces_N[index] = contact_force

        self.time_s += dt_s
        return list(self.contact_forces_N)
