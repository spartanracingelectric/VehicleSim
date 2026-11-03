"""AMK DD5-14-10-POW motor: max torque line and efficiency map.

Digitized from AMK's characteristic map (AMK_DD5_map/AMK_DD5_efficiency_map.png;
100 C motor temperature, 600 V DC bus). The efficiency is AC in -> shaft, so it
EXCLUDES inverter losses - those stay in the car's battery_to_motor_efficiency.
The data is on a 50 rpm x 0.05 Nm grid, read with bilinear interpolation. See
AMK_DD5_map/digitize_amk_map.py for how the image was turned into numbers.

    from AMK_DD5_motor import amk_dd5
    motor = amk_dd5()
    motor.efficiency(rpm, torque)              # 0-1
    motor.input_power(rpm, torque)             # AC power into one motor (W)
    motor.torque_at_input_power(rpm, p, cap)   # shaft torque one motor makes from p watts in
    motor.power_scale(rpms, torques, p_total)  # common factor to fit several motors under p_total

Outside the plotted data (below ~1.2 Nm, below ~200 rpm, above the max-torque line)
the nearest plotted efficiency is used. Generating (regen, negative torque) uses the
same map at |torque|, which assumes the motor is as efficient as a generator.
"""
from functools import lru_cache
from pathlib import Path

import numpy as np

MAP_DIR = Path(__file__).resolve().parent / "AMK_DD5_map"
RPM_TO_RAD_S = 2.0 * np.pi / 60.0


class MotorMap:
    def __init__(self, name, efficiency_csv, max_torque_csv):
        self.name = name
        data = np.genfromtxt(efficiency_csv, delimiter=",")
        self.rpm_axis = data[0, 1:]
        self.torque_axis = data[1:, 0]
        self.eta = data[1:, 1:] / 100.0                 # [torque, rpm], fraction
        line = np.genfromtxt(max_torque_csv, delimiter=",", skip_header=1)
        self.rpm_points = line[:, 0]                    # max-torque line, every 50 rpm
        self.torque_points = line[:, 1]

        # For fast single-point lookups (called every time step by the sims)
        self._rows = self.eta.tolist()
        self._dr = float(self.rpm_axis[1] - self.rpm_axis[0])
        self._dt = float(self.torque_axis[1] - self.torque_axis[0])
        self._r_last = len(self.rpm_axis) - 1
        self._t_last = len(self.torque_axis) - 1

    def __repr__(self):
        return f"MotorMap({self.name})"

    # --- efficiency ---
    def efficiency(self, rpm, torque):
        """Motor efficiency (0-1) at one speed (rpm) and shaft torque (Nm)."""
        x = abs(rpm) / self._dr
        y = abs(torque) / self._dt
        if x >= self._r_last:
            i, fx = self._r_last - 1, 1.0
        else:
            i = int(x)
            fx = x - i
        if y >= self._t_last:
            j, fy = self._t_last - 1, 1.0
        else:
            j = int(y)
            fy = y - j
        r0, r1 = self._rows[j], self._rows[j + 1]
        lo = r0[i] + (r0[i + 1] - r0[i]) * fx
        hi = r1[i] + (r1[i + 1] - r1[i]) * fx
        return lo + (hi - lo) * fy

    def efficiency_array(self, rpm, torque):
        """efficiency() for numpy arrays (same shape, or broadcastable)."""
        rpm, torque = np.broadcast_arrays(np.abs(np.asarray(rpm, float)), np.abs(np.asarray(torque, float)))
        x = np.clip(rpm / self._dr, 0.0, self._r_last)
        y = np.clip(torque / self._dt, 0.0, self._t_last)
        i = np.minimum(x.astype(int), self._r_last - 1)
        j = np.minimum(y.astype(int), self._t_last - 1)
        fx, fy = x - i, y - j
        e = self.eta
        lo = e[j, i] + (e[j, i + 1] - e[j, i]) * fx
        hi = e[j + 1, i] + (e[j + 1, i + 1] - e[j + 1, i]) * fx
        return lo + (hi - lo) * fy

    # --- power ---
    def input_power(self, rpm, torque):
        """AC power into one motor (W) making this shaft torque at this speed."""
        if torque <= 0.0 or rpm <= 0.0:
            return 0.0
        return torque * rpm * RPM_TO_RAD_S / self.efficiency(rpm, torque)

    def torque_at_input_power(self, rpm, power_in, torque_cap):
        """Most shaft torque (Nm, at most torque_cap) one motor can make at this speed
        with power_in watts of AC power going in: solves torque x speed / efficiency =
        power_in."""
        if torque_cap <= 0.0 or rpm * RPM_TO_RAD_S <= 1e-6:
            return max(torque_cap, 0.0)
        if power_in <= 0.0:
            return 0.0
        p_cap = self.input_power(rpm, torque_cap)
        if p_cap <= power_in:
            return torque_cap
        return torque_cap * _solve_scale(lambda k: self.input_power(rpm, k * torque_cap),
                                         power_in, power_in / p_cap)

    def power_scale(self, rpms, torques, power_total):
        """Factor (0-1) to scale every motor's torque by so that their combined AC input
        power fits within power_total (W). 1.0 if they already fit."""
        # Motors at the same speed and torque (left/right pairs) are looked up once
        groups = {}
        for n, t in zip(rpms, torques):
            if n > 0.0 and t > 0.0:
                groups[(n, t)] = groups.get((n, t), 0) + 1
        items = [(n, t, c) for (n, t), c in groups.items()]

        def total(k):
            return sum(c * self.input_power(n, k * t) for n, t, c in items)

        p = total(1.0)
        if p <= power_total or p <= 0.0:
            return 1.0
        if power_total <= 0.0:
            return 0.0
        return _solve_scale(total, power_total, power_total / p)

    def max_torque(self, rpm):
        """Max shaft torque (Nm) at this speed, 0 past the end of the line (20000 rpm)."""
        return float(np.interp(rpm, self.rpm_points, self.torque_points, right=0.0))


def _solve_scale(power_of, target, k):
    """Scale factor k in (0, 1] with power_of(k) = target, for power_of increasing in k
    and power_of(1) > target. Starts from k (exact if efficiency were constant), then
    secant steps - efficiency changes slowly with torque, so it takes ~2-4 lookups."""
    tol = 1e-9 * target
    g0 = power_of(k) - target
    if abs(g0) <= tol:
        return k
    k1 = k * target / (g0 + target)
    for _ in range(30):
        g1 = power_of(k1) - target
        if abs(g1) <= tol or g1 == g0:
            break
        k, g0, k1 = k1, g1, k1 - g1 * (k1 - k) / (g1 - g0)
        k1 = min(max(k1, 1e-12), 1.0)
    return k1


@lru_cache(maxsize=1)
def amk_dd5():
    """The AMK DD5-14-10-POW map (loaded once per process and shared)."""
    return MotorMap("AMK DD5-14-10-POW", MAP_DIR / "AMK_DD5_efficiency_map.csv", MAP_DIR / "AMK_DD5_max_torque.csv")
