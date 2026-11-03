"""R20 tire model whose longitudinal grip falls off past the peak slip ratio, so
wheelspin costs traction. Same interface as R20_Legacy_Tire_Model (tire_slip_ratio,
tire_forces, peak_longitudinal, peak_lateral), so a car can swap it in with
    car._tire_model = R20_V1_Tire_Model(wheel_radius=car.wheel_radius, psi=car.tire_pressure_psi)

Only the LONGITUDINAL curve is new (see fit_pacejka_falloff.py for the fit and
pacejka_r20_falloff_fit.png for how it compares with the data). Lateral force,
combined slip and TRACK_GRIP_SCALE are taken unchanged from R20_Legacy_Tire_Model.

Longitudinal curve, at normal load Fz and SAE slip ratio sr (road-speed denominator):
    D(Fz) = c1*Fz + c2*Fz^2          K(Fz) = c3*Fz + c4*Fz^2          Bx = K/D * sr
    Fx = D * sin(atan(Bx - E*(Bx - atan(Bx)))) * (1 - m*|lambda|)
lambda is the normalized slip: the sims' own slip ratio when driving (wheel-speed
denominator, 1 = full wheelspin) and the SAE slip ratio when braking (-1 = locked).

What the TTC data does and doesn't tell us:
  - The rise, the knee, the peak (~0.11-0.15 slip ratio as the sims define it) and how
    peak grip changes with load and pressure are fit to the data (R^2 0.987-0.993,
    against 0.93-0.96 for R20_Legacy_Tire_Model's longitudinal fit).
  - The TTC drive/brake sweeps stop at ~0.2 slip and are nearly flat there, so how much
    grip is left in heavy wheelspin is NOT in the data. The falloff slope m is an
    assumption: the fit is equally good for any m from 0 to 0.7. The default 0.35 leaves
    ~70% of peak grip at full wheelspin, close to published dry-road friction curves
    (Burckhardt's dry-asphalt curve keeps ~65%). Coefficients are fit separately for each
    m in LONG_PARAMS_BY_SLOPE so the curve still matches the data whichever is used.

Slip conventions: the TTC SR channel is SAE (road-speed denominator) and reads about
-0.025 at free rolling because it uses the loaded radius; the fit removes that offset.
The sims' slip ratio uses the wheel-speed denominator, so it's converted here
(sr = kappa / (1 - kappa)) rather than used as if it were the SAE value, which is what
R20_Legacy_Tire_Model does.

Fit loads were 300-1900 N. Below ~300 N the TTC data shows lower grip than this
quadratic D(Fz) gives.
"""
import math
from functools import lru_cache

import numpy as np

from R20_Legacy_Tire_Model import R20_Legacy_Tire_Model

# (c1, c2, c3, c4, E) per falloff slope m and pressure (psi), from fit_pacejka_falloff.py.
# Raw-point R^2 at 8/10/12/14 psi: 0.987 / 0.993 / 0.988 / 0.992 (all four m values).
LONG_PARAMS_BY_SLOPE = {
    0.0: {
        8.0: (2.6983911, -0.00017740415, 51.868271, -0.0099160609, -2.1285737),
        10.0: (2.4581954, -0.00014931237, 41.905709, -0.0022876965, -1.6584278),
        12.0: (2.4933656, -0.00022219301, 35.019064, 0.0041621009, -1.7792581),
        14.0: (2.2170355, -9.7570523e-05, 29.211314, 0.005173539, -1.0482326),
    },
    0.2: {
        8.0: (2.7773963, -0.00017693372, 52.465058, -0.0098961142, -1.883492),
        10.0: (2.5393961, -0.00015506493, 42.505472, -0.0022446513, -1.414265),
        12.0: (2.5816257, -0.00023721214, 35.505636, 0.0042394799, -1.5436623),
        14.0: (2.303472, -0.00010920859, 29.54767, 0.005204733, -0.88842591),
    },
    0.35: {
        8.0: (2.8412674, -0.00017658136, 52.914162, -0.0098824028, -1.7067217),
        10.0: (2.6061452, -0.00016028603, 42.94451, -0.002200851, -1.2377793),
        12.0: (2.6539676, -0.00025007158, 35.863259, 0.0043053393, -1.3728941),
        14.0: (2.3753313, -0.00011966444, 29.791071, 0.0052334668, -0.76999245),
    },
    0.5: {
        8.0: (2.9096558, -0.00017620557, 53.364499, -0.0098704758, -1.5354011),
        10.0: (2.6788595, -0.00016648419, 43.370692, -0.0021455701, -1.0664116),
        12.0: (2.7324933, -0.00026458731, 36.212306, 0.0043784622, -1.2069209),
        14.0: (2.4544342, -0.0001319631, 30.023448, 0.0052685128, -0.65234785),
    },
}
DEFAULT_FALLOFF_SLOPE = 0.35


class R20_V1_Tire_Model:
    TRACK_GRIP_SCALE = R20_Legacy_Tire_Model.TRACK_GRIP_SCALE

    def __init__(self, wheel_radius=0.203, psi=12.0, falloff_slope=DEFAULT_FALLOFF_SLOPE):
        if falloff_slope not in LONG_PARAMS_BY_SLOPE:
            raise ValueError(f"falloff_slope must be one of {sorted(LONG_PARAMS_BY_SLOPE)} "
                             "(each needs its own fit - see fit_pacejka_falloff.py)")
        self.wheel_radius = wheel_radius
        self.psi = psi
        self.falloff_slope = falloff_slope
        self._lateral = R20_Legacy_Tire_Model(wheel_radius=wheel_radius, psi=psi)
        self.long_params = self._interp_params(psi, LONG_PARAMS_BY_SLOPE[falloff_slope])

    @classmethod
    def _interp_params(cls, psi, params_by_psi):
        """Linear in each coefficient between the bracketing tested pressures (clamped to
        8-14 psi), then TRACK_GRIP_SCALE on the peak-force terms, as R20_Legacy_Tire_Model does."""
        psis = sorted(params_by_psi)
        p = min(max(psi, psis[0]), psis[-1])
        lo = max(q for q in psis if q <= p)
        hi = min(q for q in psis if q >= p)
        t = 0.0 if hi == lo else (p - lo) / (hi - lo)
        c1, c2, c3, c4, E = np.array(params_by_psi[lo]) + t * (np.array(params_by_psi[hi]) - np.array(params_by_psi[lo]))
        return (float(c1 * cls.TRACK_GRIP_SCALE), float(c2 * cls.TRACK_GRIP_SCALE), float(c3), float(c4), float(E))

    def tire_slip_ratio(self, wheel_omega, vx):
        """Same definition the sims already use (wheel-speed denominator, clamped)."""
        return self._lateral.tire_slip_ratio(wheel_omega, vx)

    def _fx(self, fz, kappa):
        """Pure longitudinal force (N) at the sims' slip ratio kappa."""
        c1, c2, c3, c4, E = self.long_params
        D = c1 * fz + c2 * fz * fz
        if D <= 0.0:
            return 0.0
        sr = kappa / (1.0 - kappa)                    # sims' slip ratio -> SAE slip ratio
        lam = kappa if kappa >= 0.0 else sr           # 1 = full wheelspin, -1 = locked
        Bx = (c3 * fz + c4 * fz * fz) / D * sr
        rise = math.sin(math.atan(Bx - E * (Bx - math.atan(Bx))))
        return D * rise * (1.0 - self.falloff_slope * abs(lam))

    def peak_longitudinal(self, fz):
        """Highest |Fx| (N) the curve reaches at this load (driving side)."""
        if fz <= 0.0:
            return 0.0
        return _peak_fx(self.long_params, self.falloff_slope, round(float(fz), 1))

    def peak_slip_ratio(self, fz):
        """Slip ratio (sims' definition) where driving force peaks at this load."""
        return _peak_kappa(self.long_params, self.falloff_slope, round(float(fz), 1))

    def peak_lateral(self, fz):
        return self._lateral.peak_lateral(fz)

    def tire_forces(self, fz, kappa, alpha=0.0):
        """(Fx, Fy) in N. Pure slip uses each axis's own curve; combined slip scales both
        down to stay inside a friction ellipse sized by each axis's peak (as R20_Legacy_Tire_Model)."""
        if fz <= 0.0:
            return 0.0, 0.0
        fx = self._fx(fz, kappa)
        if alpha == 0.0:
            return fx, 0.0
        fy = self._lateral.tire_forces(fz, 0.0, alpha)[1]
        dx, dy = self.peak_longitudinal(fz), self.peak_lateral(fz)
        r = math.hypot(fx / dx if dx else 0.0, fy / dy if dy else 0.0)
        if r > 1.0:
            fx, fy = fx / r, fy / r
        return fx, fy


def _curve(params, m, fz, kappas):
    c1, c2, c3, c4, E = params
    D = c1 * fz + c2 * fz * fz
    Bx = (c3 * fz + c4 * fz * fz) / D * kappas / (1.0 - kappas)
    return D * np.sin(np.arctan(Bx - E * (Bx - np.arctan(Bx)))) * (1.0 - m * kappas)


@lru_cache(maxsize=4096)
def _peak_kappa(params, m, fz):
    k = np.linspace(1e-4, 0.9, 9000)
    return float(k[np.argmax(_curve(params, m, fz, k))])


@lru_cache(maxsize=4096)
def _peak_fx(params, m, fz):
    return float(_curve(params, m, fz, np.array([_peak_kappa(params, m, fz)]))[0])
