"""Hoosier LC0 16.0x7.5-10 tire model, built the same way as the R20 falloff model
(Tire_R20/R20_V1_Tire_Model.py) and with the same interface, so a car can swap it in:
    car._tire_model = LC0_V1_Tire_Model(wheel_radius=car.wheel_radius, psi=car.tire_pressure_psi)

Data (Calspan TTC; build_dataset.py lists the runs and why each was picked):
  - Lateral: Round 8 cornering runs on the 16x7.5-10 LC0 itself (7" and 8" rims),
    8/10/12/14 psi. R^2 0.974 / 0.979 / 0.987 / 0.984.
  - Longitudinal: no 16" LC0 was ever drive/brake tested, so (as for the R20) the curve
    comes from the same compound on another carcass, the 18.0x6.0-10 LC0 (Rounds 5 and 6,
    6" and 7" rims). R^2 0.987 / 0.989 / 0.963 / 0.993 at 8/10/12/14 psi.

Model, as the R20 falloff model (fit_pacejka_lc0.py does the fitting):
  - Longitudinal: C = 1, E <= 1 Magic Formula with the TTC slip offset removed, times
    (1 - m*|lambda|) so grip falls off with wheelspin. The data stops at ~0.18 slip, so
    the falloff slope m is assumed (default 0.35, ~70% of peak at full wheelspin), with a
    separate fit per m.
  - One difference from the R20: braking peak grip is fitted separately, as r x the
    driving peak (r = 1.07-1.28; same slope at zero slip). The LC0 data grips harder
    braking than driving, and one symmetric curve over-predicted driving grip by 5-16%.
    With r, the driving side is within +-2% of the data. (The R20 data doesn't need
    this: its symmetric fit is within +-5% on both sides.)
  - Lateral: R20_Legacy_Tire_Model's 6-coefficient form, held flat past its peak.
  - Combined slip: the same friction-ellipse scaling.
  - TRACK_GRIP_SCALE = 0.65 (TTC belt -> real track) is reused from the R20, where it was
    calibrated against SR_17's real Endurance lap. It has not been checked for the LC0.

One addition: the fitted load terms are only trusted up to the highest tested load
(FZ_FIT_MAX). Above it the curve shape is held at FZ_FIT_MAX and force scales in
proportion to load. Without that, the 8 psi longitudinal stiffness term reaches zero
near 2000 N. Late in a run the Accel sims reach ~1300-1500 N, slightly past this.

Caveats: the longitudinal (Rounds 5/6, 2011-12) and lateral (Round 8, 2018) data are from
different test years, and the R20 data from Round 9, so belt condition can differ
between them. At ~1050 N the LC0 drive data drops a little between 0.15 and 0.20
slip, more than the fitted curve does.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Tire_R20"))
from R20_Legacy_Tire_Model import R20_Legacy_Tire_Model  # noqa: E402
from R20_V1_Tire_Model import R20_V1_Tire_Model, DEFAULT_FALLOFF_SLOPE  # noqa: E402

FZ_FIT_MAX = 1250.0      # N; highest load in the drive/brake data

# (c1, c2, c3, c4, E, r) per falloff slope m and pressure (psi), from fit_pacejka_lc0.py.
# r = braking/driving peak ratio (braking D = r x driving D).
LONG_PARAMS_BY_SLOPE = {
    0.0: {
        8.0: (2.5765436, -0.00042145662, 76.600905, -0.039232271, -2.0539447, 1.260152),
        10.0: (2.631152, -0.00042494486, 66.824965, -0.030924489, -1.2076849, 1.1022849),
        12.0: (2.6710412, -0.0003934398, 46.6633, -0.012472957, -2.9718519, 1.0580624),
        14.0: (2.3146921, -0.00042510321, 51.280857, -0.017714686, -1.0126955, 1.2128826),
    },
    0.2: {
        8.0: (2.6251075, -0.00040303448, 77.624127, -0.039682345, -1.7995049, 1.2707659),
        10.0: (2.694202, -0.00041268226, 67.596095, -0.031199563, -0.99808026, 1.1110441),
        12.0: (2.7456687, -0.00040213015, 47.266384, -0.012450161, -2.6466058, 1.0657448),
        14.0: (2.3802761, -0.00042580283, 52.12855, -0.018076741, -0.79249914, 1.2247012),
    },
    0.35: {
        8.0: (2.6623698, -0.00038608751, 78.418517, -0.040031085, -1.6102142, 1.2795596),
        10.0: (2.7439656, -0.00040045995, 68.161325, -0.03139649, -0.84401576, 1.1182909),
        12.0: (2.8056495, -0.00040896976, 47.72085, -0.012426933, -2.4104654, 1.07197),
        14.0: (2.4331436, -0.00042500166, 52.747718, -0.018344459, -0.63296397, 1.2346311),
    },
    0.5: {
        8.0: (2.7002939, -0.00036580267, 79.239687, -0.040392763, -1.4216888, 1.2892099),
        10.0: (2.7960623, -0.00038487598, 68.714979, -0.031589211, -0.69244139, 1.1262276),
        12.0: (2.8695301, -0.00041608938, 48.176327, -0.012399269, -2.1811244, 1.0786583),
        14.0: (2.4897775, -0.0004226769, 53.350267, -0.018611477, -0.47748209, 1.2456637),
    },
}

# (C, c1, c2, c3, c4, E) per pressure, slip angle in radians, from fit_pacejka_lc0.py
LAT_PARAMS_BY_PSI = {
    8.0: (2.2808212, -2.9309794, 0.00054459133, -48.980954, 0.019104796, 1.4827156),
    10.0: (1.8918097, -2.8765925, 0.00037752422, -47.701868, 0.016220257, 1.2019407),
    12.0: (1.565245, -2.4794681, 0.000202249, -38.671957, 0.0084709476, 0.29831997),
    14.0: (1.2822654, -2.4742954, 0.00025808688, -38.276978, 0.0093737638, 0.30281039),
}


def _load_clamp(fz):
    """(load to evaluate the fitted curve at, factor to scale the force by)."""
    f = min(fz, FZ_FIT_MAX)
    return f, fz / f


class _LC0Lateral(R20_Legacy_Tire_Model):
    """R20_Legacy_Tire_Model (flat hold past the peak) with the LC0 lateral coefficients. Its
    longitudinal side is never used: LC0_V1_Tire_Model supplies Fx."""
    LAT_PARAMS_BY_PSI = LAT_PARAMS_BY_PSI

    def _pure(self, slip, fz, params, x_peak):
        f, scale = _load_clamp(fz)
        return super()._pure(slip, f, params, x_peak) * scale

    def _peak(self, fz, params, x_peak):
        f, scale = _load_clamp(fz)
        return super()._peak(f, params, x_peak) * scale


class LC0_V1_Tire_Model(R20_V1_Tire_Model):
    NAME = "LC0"
    FZ_FIT_MAX = FZ_FIT_MAX          # read by the dynamic model (Dynamics/solver/vehicle_model.py)

    def __init__(self, wheel_radius=0.203, psi=12.0, falloff_slope=DEFAULT_FALLOFF_SLOPE):
        if falloff_slope not in LONG_PARAMS_BY_SLOPE:
            raise ValueError(f"falloff_slope must be one of {sorted(LONG_PARAMS_BY_SLOPE)}")
        self.wheel_radius = wheel_radius
        self.psi = psi
        self.falloff_slope = falloff_slope
        self._lateral = _LC0Lateral(wheel_radius=wheel_radius, psi=psi)
        table = LONG_PARAMS_BY_SLOPE[falloff_slope]
        # (c1, c2, c3, c4, E) go through the R20 class's interpolation + grip scaling;
        # the brake/drive ratio r is interpolated the same way, unscaled
        self.long_params = self._interp_params(psi, {p: v[:5] for p, v in table.items()})
        psis = sorted(table)
        self.brake_ratio = float(np.interp(min(max(psi, psis[0]), psis[-1]), psis, [table[p][5] for p in psis]))

    def _fx(self, fz, kappa):
        f, scale = _load_clamp(fz)
        fx = super()._fx(f, kappa)
        if kappa < 0.0:      # braking side: peak r x higher, same slope at zero slip
            c1, c2, c3, c4, E = self.long_params
            r = self.brake_ratio
            D = r * (c1 * f + c2 * f * f)
            if D <= 0.0:
                return 0.0
            sr = kappa / (1.0 - kappa)
            Bx = (c3 * f + c4 * f * f) / D * sr
            fx = D * math.sin(math.atan(Bx - E * (Bx - math.atan(Bx)))) * (1.0 - self.falloff_slope * abs(sr))
        return fx * scale

    def peak_longitudinal(self, fz):
        """Highest driving |Fx| (N) at this load (braking peak is brake_ratio x this)."""
        if fz <= 0.0:
            return 0.0
        f, scale = _load_clamp(fz)
        return super().peak_longitudinal(f) * scale

    def peak_slip_ratio(self, fz):
        return super().peak_slip_ratio(min(fz, FZ_FIT_MAX))

    def tire_forces(self, fz, kappa, alpha=0.0):
        """As R20_V1_Tire_Model.tire_forces, with the ellipse's longitudinal size
        taken from the braking peak when braking."""
        if fz <= 0.0:
            return 0.0, 0.0
        fx = self._fx(fz, kappa)
        if alpha == 0.0:
            return fx, 0.0
        fy = self._lateral.tire_forces(fz, 0.0, alpha)[1]
        dx = self.peak_longitudinal(fz) * (self.brake_ratio if kappa < 0.0 else 1.0)
        dy = self.peak_lateral(fz)
        rr = math.hypot(fx / dx if dx else 0.0, fy / dy if dy else 0.0)
        if rr > 1.0:
            fx, fy = fx / rr, fy / rr
        return fx, fy
