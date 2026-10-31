"""Small backward-Euler first-order low-pass filter."""


class FirstOrderFilter:
    """Filter ``y`` toward ``u`` with time constant ``tau_s``.

    The update is the stable backward-Euler form
    ``y[k] = y[k-1] + dt / (tau + dt) * (u[k] - y[k-1])``.
    """

    def __init__(self, tau_s: float, initial_value: float = 0.0):
        if tau_s < 0:
            raise ValueError(f"tau_s must be >= 0, got {tau_s}")
        self.tau_s = float(tau_s)
        self.value = float(initial_value)
        self.initialized = False

    def reset(self, value: float = 0.0) -> None:
        self.value = float(value)
        self.initialized = False

    def update(self, input_value: float, dt_s: float) -> float:
        if dt_s <= 0:
            raise ValueError(f"dt_s must be > 0, got {dt_s}")
        if not self.initialized:
            self.value = float(input_value)
            self.initialized = True
            return self.value
        if self.tau_s == 0:
            self.value = float(input_value)
        else:
            alpha = dt_s / (self.tau_s + dt_s)
            self.value += alpha * (float(input_value) - self.value)
        return self.value
