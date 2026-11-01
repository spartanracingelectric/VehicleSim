"""Hoosier LC0 16.0x7.5-10 tire model built on LC0_V1_Tire_Model the way R20_V2_Tire_Model is
built on R20_V1: a refit lateral curve with camber, and measured combined slip (ideas from the
oncle_lapsim toolbox's MF 6.2 tire model). Same interface as R20_V2_Tire_Model:
    car._tire_model = LC0_V2_Tire_Model(wheel_radius=car.wheel_radius, psi=car.tire_pressure_psi)

Fit by fit_lc0_v2.py (see lc0_v2_fit.png); compared with LC0_V1 by compare_lc0_v2_vs_v1.py.
It is the "LC0_V2" tire choice in the Accel sims. (Fitting scripts and data: Simulations/Tire_LC0.)

What's the same as LC0_V1_Tire_Model
  - Pure longitudinal force: its exact curve, including the separate braking peak (r x the
    driving peak), the falloff past the peak, and the load clamp above FZ_FIT_MAX (1250 N).
  - TRACK_GRIP_SCALE = 0.65 on the peak-force terms (never checked for the LC0).
  - With alpha = 0 and camber 0, tire_forces() returns exactly what LC0_V1 does, so the Accel
    sims give identical results.

What's new
  1. Lateral curve, from the same 16x7.5-10 LC0 cornering runs (Round 8, runs 15, 16, 18, 19):
       - camber fitted rather than pooled: peak loss (~1% at 2 deg, 2-5% at 4 deg), stiffness
         loss and camber thrust (optional camber_deg argument, default 0);
       - MF 6.x stiffness form, which levels off with load but never reverses, so no load clamp
         is needed on the lateral side (peak force turns over at 2600-4800 N, well past the
         1681 N the data reaches);
       - the tire's own small zero-camber offsets are dropped (left/right symmetric).
     RMSE on the data drops from 143/131/120/101 N to 90/83/103/74 N at 8/10/12/14 psi. At
     camber 0 alone the two are about equal: the gain is in handling camber.
  2. Combined slip from data, not a friction ellipse: the MF 6.1/6.2 weighting functions
        Fx = Gxa(kappa, alpha) * Fx_pure(kappa)      Fy = Gyk(kappa, alpha) * Fy_pure(alpha)
     fit to the 18x6.0-10 LC0 drive/brake runs at -3/-6 deg slip angle (Rounds 5 and 6).
     Given the same pure-slip forces, the friction ellipse leaves 228 N (Fx) and 319 N (Fy)
     RMSE on that data; MF leaves 103 N and 114 N. Above 0.08 slip ratio the ellipse overstates
     side force by 50% on average (median 35%); MF is within 1%.

Things to know
  - No 16" LC0 was drive/brake tested, so the combined-slip shape (like the longitudinal curve)
    comes from the 18x6.0-10 LC0. It holds across rim widths and test rounds, but there is no
    second carcass size to check it against, as there was for the R20.
  - The Round 5/6 sweeps make zero drive force at a slightly different slip ratio at -3/-6 deg
    than at 0 deg. That is treated as a measurement offset (see fit_lc0_v2.py).
  - Combined data covers 3-6 deg slip angle, SAE slip ratio -0.24 to +0.22 and 630-1290 N
    (lab). Past that, Gxa and Gyk are Magic Formula extrapolation.
  - The combined-slip functions are evaluated at slip / TRACK_GRIP_SCALE, as in R20_V2.
  - The longitudinal (2011-12) and lateral (2018) data are from different test years.
  - Camber sign is the TTC/SAE inclination angle, as in R20_V2.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Tire_R20"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from LC0_V1_Tire_Model import LC0_V1_Tire_Model, DEFAULT_FALLOFF_SLOPE   # noqa: E402
from R20_V2_Tire_Model import _shape, _x_peak, _weight                 # noqa: E402  (same MF helpers)

# Lateral, per pressure, lab surface: (C, c1, c2, kp, kf, E, d3, k3, h3, v3), same form as R20_V2:
#   D = (c1*Fz + c2*Fz^2)*(1 - d3*g^2)   K = kp*sin(2*atan(Fz/kf))*(1 - k3*|g|)   B = K/(C*D)
#   x = B*(alpha + h3*g)   Fy = D*sin(C*atan(x - E*(x - atan(x)))) + Fz*v3*g     (alpha, g in rad)
# Fit on 16x7.5-10 LC0 runs 15, 16, 18, 19. Fit R^2 0.9936 / 0.9944 / 0.9924 / 0.9953.
LAT_PARAMS_BY_PSI = {
    8.0: (1.6607572, -2.9624049, 0.00057146722, -32417.97, 1405.2416, 0.40252649, 10.163885, 0.44360499, -0.21502869, -1.0306281),
    10.0: (1.4479508, -2.9118234, 0.0004383615, -37310.563, 1648.4161, 0.34210692, 8.8078585, 1.1780333, -0.20055069, -1.2492333),
    12.0: (1.7791816, -2.5282592, 0.00026285962, -47160.788, 2692.2163, 0.31521655, 4.1533439, 0.92284994, -0.17713207, -1.1591095),
    14.0: (1.2583094, -2.5932224, 0.00034783505, -43701.402, 2434.4844, 0.31225259, 9.3542345, 1.1105621, -0.16039443, -0.87116047),
}
# Combined slip (MF 6.1/6.2), lab surface; kappa = SAE slip ratio, alpha in rad, dfz = (Fz - 1000)/1000.
GXA_PARAMS = (21.644503, 16.136252, 0.89811543, 0.31712633, 1.3107132)    # rBx1, rBx2, rCx1, rEx1, rEx2
GYK_PARAMS = (12.779823, 12.66736, 1.0249139, -0.39664233, -0.91632717, 0.035880535, 0.046584904)
#             rBy1,      rBy2,     rCy1,      rEy1,        rEy2,        rHy1,        rHy2
FZ_NOM = 1000.0
DATA_MAX_FZ_LATERAL = 1681.0        # N, highest load in the 16x7.5-10 LC0 cornering data


class LC0_V2_Tire_Model:
    NAME = "LC0"
    TRACK_GRIP_SCALE = LC0_V1_Tire_Model.TRACK_GRIP_SCALE
    GXA_PARAMS, GYK_PARAMS, FZ_NOM = GXA_PARAMS, GYK_PARAMS, FZ_NOM

    def __init__(self, wheel_radius=0.203, psi=12.0, falloff_slope=DEFAULT_FALLOFF_SLOPE, grip_scale=None):
        """grip_scale: None = TRACK_GRIP_SCALE (track, what the sims use); 1.0 = TTC lab surface."""
        self.wheel_radius = wheel_radius
        self.psi = psi
        self.falloff_slope = falloff_slope
        self.grip_scale = self.TRACK_GRIP_SCALE if grip_scale is None else float(grip_scale)
        long_cls = LC0_V1_Tire_Model
        if self.grip_scale != LC0_V1_Tire_Model.TRACK_GRIP_SCALE:
            long_cls = type("LC0_V1_Tire_Model_scaled", (LC0_V1_Tire_Model,), {"TRACK_GRIP_SCALE": self.grip_scale})
        self._long = long_cls(wheel_radius=wheel_radius, psi=psi, falloff_slope=falloff_slope)
        self.brake_ratio = self._long.brake_ratio
        self.lat_params = self._interp_lat(psi)
        self._xp = _x_peak(self.lat_params[0], self.lat_params[5])

    def _interp_lat(self, psi):
        """Linear in each coefficient between the bracketing tested pressures (clamped to 8-14
        psi), then the grip scale on the force terms (c1, c2 and camber thrust v3)."""
        psis = sorted(LAT_PARAMS_BY_PSI)
        p = min(max(psi, psis[0]), psis[-1])
        lo = max(q for q in psis if q <= p)
        hi = min(q for q in psis if q >= p)
        t = 0.0 if hi == lo else (p - lo) / (hi - lo)
        q = np.array(LAT_PARAMS_BY_PSI[lo]) + t * (np.array(LAT_PARAMS_BY_PSI[hi]) - np.array(LAT_PARAMS_BY_PSI[lo]))
        s = self.grip_scale
        C, c1, c2, kp, kf, E, d3, k3, h3, v3 = (float(v) for v in q)
        return (C, c1 * s, c2 * s, kp, kf, E, d3, k3, h3, v3 * s)

    # ------------------------------------------------------------ same as LC0_V1
    def tire_slip_ratio(self, wheel_omega, vx):
        return self._long.tire_slip_ratio(wheel_omega, vx)

    def peak_longitudinal(self, fz):
        """Highest driving |Fx| (N) at this load (braking peak is brake_ratio x this)."""
        return self._long.peak_longitudinal(fz)

    def peak_slip_ratio(self, fz):
        return self._long.peak_slip_ratio(fz)

    # ------------------------------------------------------------ lateral
    def _fy_pure(self, fz, alpha, g):
        """Pure lateral force (N); alpha, g (camber) in rad. Held at the curve's own peak past
        the peak slip angle."""
        C, c1, c2, kp, kf, E, d3, k3, h3, v3 = self.lat_params
        D = (c1 * fz + c2 * fz * fz) * (1.0 - d3 * g * g)
        if D == 0.0:
            return 0.0
        K = kp * math.sin(2.0 * math.atan(fz / kf)) * (1.0 - k3 * abs(g))
        x = max(-self._xp, min(self._xp, K / (C * D) * (alpha + h3 * g)))
        return D * _shape(x, C, E) + fz * v3 * g

    def peak_lateral(self, fz, camber_deg=0.0):
        """Highest |Fy| (N) at this load. With camber, the larger of the two sides."""
        if fz <= 0.0:
            return 0.0
        C, c1, c2, _, _, E, d3, _, _, v3 = self.lat_params
        g = math.radians(camber_deg)
        D = (c1 * fz + c2 * fz * fz) * (1.0 - d3 * g * g)
        return abs(D) * _shape(self._xp, C, E) + abs(fz * v3 * g)

    # ------------------------------------------------------------ combined
    def combined_weights(self, fz, kappa, alpha):
        """(Gxa, Gyk): the fractions of pure-slip Fx and Fy left at this slip ratio (sims'
        convention) and slip angle (rad). Slip is divided by the grip scale, as in R20_V2."""
        s = self.grip_scale
        sr = kappa / (1.0 - kappa) / s                # sims' slip ratio -> SAE, normalized by grip
        a = abs(alpha) / s
        dfz = (fz - FZ_NOM) / FZ_NOM
        rBx1, rBx2, rCx1, rEx1, rEx2 = GXA_PARAMS
        gxa = _weight(rBx1 * math.cos(math.atan(rBx2 * sr)), rCx1, min(rEx1 + rEx2 * dfz, 1.0), a)
        rBy1, rBy2, rCy1, rEy1, rEy2, rHy1, rHy2 = GYK_PARAMS
        B = rBy1 * math.cos(math.atan(rBy2 * a))
        E = min(rEy1 + rEy2 * dfz, 1.0)
        sh = rHy1 + rHy2 * dfz
        gyk = _weight(B, rCy1, E, sr + sh) / _weight(B, rCy1, E, sh)
        return max(gxa, 0.0), max(gyk, 0.0)

    def tire_forces(self, fz, kappa, alpha=0.0, camber_deg=0.0):
        """(Fx, Fy) in N. kappa: the sims' slip ratio; alpha: slip angle (rad, SAE sign: positive
        alpha gives negative Fy); camber_deg: inclination angle (deg, SAE).
        With alpha = 0 and no camber this is exactly LC0_V1_Tire_Model.tire_forces()."""
        if fz <= 0.0:
            return 0.0, 0.0
        fx = self._long._fx(fz, kappa)
        if alpha == 0.0 and camber_deg == 0.0:
            return fx, 0.0
        fy = self._fy_pure(fz, alpha, math.radians(camber_deg))
        if kappa == 0.0 and alpha == 0.0:
            return fx, fy
        gxa, gyk = self.combined_weights(fz, kappa, alpha)
        return fx * gxa, fy * gyk
