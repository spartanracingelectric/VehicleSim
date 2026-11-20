import math
import numpy as np

from Vehicle.Tire.PacejkaR20TireConfig import PacejkaR20TireConfig

# Curve equation = mu(s) = c1 * (1-exp(-c2 * slip)) - c3 * slip
# c1 | c2 | c3 are tunable

SURFACES = {
    "dry_asphalt": (1.2801, 23.99, 0.52),
    "dry_concrete": (1.1973, 25.168, 0.5373),
    # TODO: Add one for gravelly surfaces (PNR)
}

# Equation

def burckhardt_mu(slip, c1, c2, c3):
    return c1 * (1-math.exp(-c2 * slip)) - c3 * slip

# Updates

class BurckhardtR20TireConfig(PacejkaR20TireConfig):
    model_name = "pacejka_r20_burckhardt"
    def __init__(self, radius_m=0.2032, psi=12.0, track_grip_scale=0.65, surface="dry_asphalt"):
        super().__init__(radius_m=radius_m, psi=psi, track_grip_scale=track_grip_scale)
        self.surface = surface 
        self.c1, self.c2, self.c3 = SURFACES[surface]

        self.fade_start_slip = math.log(self.c1 * self.c2 / self.c3) / self.c2
        self.peak_mu = burckhardt_mu(self.fade_start_slip, self.c1, self.c2, self.c3)
        self._past_peak = False

# Slip curve parallel to grip loss

    def fade(self, slip):
        # 1.0 = full grip
        s = min(abs(slip), 1.0)
        if s <= self.fade_start_slip: # begins at peak slip which is 0.17
            return 1.0                # if 1.0, then no penalty is applied
        return burckhardt_mu(s, self.c1, self.c2, self.c3) / self.peak_mu

# Override pacejka

    def _pure(self, slip, fz, params, x_peak):
        if params is not self.long_params:
            return super()._pure(slip, fz, params, x_peak)
        C, c1, c2, c3, c4, E = params
        D = c1 * fz + c2 * fz * fz
        K = c3 * fz + c4 * fz * fz
        D_safe = np.sign(D) * max(abs(D), 1.0) if D != 0 else 1.0
        B = K / (C * D_safe)
        x = B * slip
        if abs(x) <= x_peak:
            return D * self._shape(x, C, E)
        self._past_peak = True
        return np.sign(x) * D * self._shape(x_peak, C, E) * self.fade(slip)

    def calculateForces(self, Fz_N, slip_ratio, slip_angle_rad, dt_s=None):
        self._past_peak = False
        return super().calculateForces(Fz_N, slip_ratio, slip_angle_rad)

    def isAtGripLimit(self):
        return self._past_peak or super().isAtGripLimit()
