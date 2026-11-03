"""R20 tire model built on R20_V1_Tire_Model, with a refit lateral curve and real combined slip.
Same interface as R20_V1_Tire_Model (tire_slip_ratio, tire_forces, peak_longitudinal,
peak_lateral, peak_slip_ratio), so a car can swap it in with
    car._tire_model = R20_V2_Tire_Model(wheel_radius=car.wheel_radius, psi=car.tire_pressure_psi)

Fit by fit_pacejka_combined.py (see pacejka_r20_combined_fit.png). Ideas taken from the
oncle_lapsim toolbox's MF 6.2 tire model (a port of MFeval); the code here is our own.

What's the same as R20_V1_Tire_Model
  - Pure longitudinal force: its exact curve and coefficients, including the falloff slope m
    past the peak. Camber was checked and left out: it made no consistent difference to Fx.
  - TRACK_GRIP_SCALE = 0.65 on the peak-force terms, and the slip-ratio conventions.
  - With alpha = 0 and camber 0, tire_forces() returns exactly what the falloff model does. So
    the Accel sims, which never steer, give identical results.

What's new
  1. Lateral curve fit to the car's own tire, the 16x7.5-10. It is fit to 232k points from
     runs 4, 5, 6, 8 and 9. R20_Legacy_Tire_Model pools all four R20 sizes, and the 18x6 grips ~8% more.
     On the 16x7.5 data the RMSE drops from 155/122/99/88 N to 105/80/69/57 N at 8/10/12/14 psi.
     That is the model as used: symmetric, and held at its peak. The fit itself, before its small
     offsets are dropped, reaches 101/78/66/54 N.
  2. Camber (optional camber_deg argument, default 0). It does three things:
       - It costs peak grip: about 1.5% at 2 deg and 4-7% at 4 deg.
       - It softens cornering stiffness.
       - It adds camber thrust: about 0.27*Fz of side force at 4 deg with no slip angle, on the
         lab surface.
     The tire's own small zero-camber offsets (ply steer and conicity) are dropped, so the curve
     is left/right symmetric.
  3. Combined slip from data, not a friction circle.
       - The TTC drive/brake runs swept slip ratio at 0, -3 and -6 deg slip angle. The -3/-6 deg
         parts (about two thirds of those runs) had never been used.
       - They are fit with the Magic Formula 6.1/6.2 weighting functions:
            Fx = Gxa(kappa, alpha) * Fx_pure(kappa)
            Fy = Gyk(kappa, alpha) * Fy_pure(alpha)
       - On that data, given the same pure-slip forces, the falloff model's friction ellipse
         leaves 264 N (Fx) and 267 N (Fy) RMSE; the MF weighting leaves 158 N and 115 N.
       - The friction ellipse overstates the side force left while the tire is driving or
         braking hard (|slip ratio| > 0.08) by 37% on average; MF is within 1%.
       - The fit carries over between tire sizes: fit on the 18x6 and tested on the 20.5x7, or
         the reverse, it still beats the ellipse on both forces.

Things to know
  - Only the 18x6.0-10 and 20.5x7.0-13 were drive/brake tested. The combined-slip shape is
    assumed to carry over to the 16x7.5, as the longitudinal curve already is.
  - Combined data covers slip angles of 3-6 deg and slip ratios of about -0.3 to +0.23 (lab).
    Past that, Gxa and Gyk are Magic Formula extrapolation:
       - At very large slip angles Gxa keeps falling.
       - In heavy wheelspin Gyk takes most of the side force away (a fully spinning tire has
         little cornering grip left).
       - The ellipse keeps about 70% of the side force there.
  - Track surface. TRACK_GRIP_SCALE lowers peak force but not stiffness, so each pure curve
    peaks at a smaller slip on track than in the lab. The combined-slip functions are scaled to
    match: they are evaluated at slip / TRACK_GRIP_SCALE. That is the brush-model result that the
    slip where a tire starts sliding scales with friction. With grip_scale=1 the model is exactly
    the lab fit.
  - TRACK_GRIP_SCALE was calibrated with R20_Legacy_Tire_Model's pooled lateral curve, which averages
    0/2/4 deg camber. At 0 deg camber the new peak lateral grip, compared with the old curve:
       - 300-1100 N: within -1% to +7%; the gain grows with load and as pressure drops.
       - 1400-1800 N (beyond the 16x7.5 data): 3-10% higher.
    At 2 deg camber it is within -1% to +5% of the old curve. Any sim moved onto this model
    should have the scale rechecked.
  - Lateral data reaches 1264 N on the 16x7.5. Above that the load curve is extrapolated.
  - Camber sign is the TTC/SAE inclination angle: positive = top of the tire leaning toward +y,
    the same axis Fy uses. Camber thrust pushes toward the side the top leans to.
  - Not modelled (Oncle's MF 6.2 has these; nothing in our sims uses them): aligning moment Mz,
    overturning and rolling moments, turn slip, relaxation lengths.
"""
import math
from functools import lru_cache

import numpy as np

from R20_V1_Tire_Model import R20_V1_Tire_Model, DEFAULT_FALLOFF_SLOPE

# Lateral, per pressure, lab surface: (C, c1, c2, kp, kf, E, d3, k3, h3, v3)
#   D = (c1*Fz + c2*Fz^2)*(1 - d3*g^2)   K = kp*sin(2*atan(Fz/kf))*(1 - k3*|g|)   B = K/(C*D)
#   x = B*(alpha + h3*g)   Fy = D*sin(C*atan(x - E*(x - atan(x)))) + Fz*v3*g     (alpha, g in rad)
# Fit on 16x7.5-10 runs 4, 5, 6, 8, 9. Raw-point R^2 0.9958 / 0.9974 / 0.9981 / 0.9986.
# Stiffness uses the MF 6.x load form (PKY1/PKY2): it levels off with load but never reverses.
# A quadratic fit crossed zero at ~1750 N, just past the data.
LAT_PARAMS_BY_PSI = {
    8.0: (1.6142438, -2.9040974, 0.0003755268, -31960.16, 884.6638, 0.25787763, 14.261628, 0.9667541, -0.13730209, 0.18500769),
    10.0: (1.6983803, -2.6507342, 0.0001882929, -35360.829, 1267.1844, 0.25911819, 10.560187, 0.70058621, -0.11853503, 0.49567983),
    12.0: (1.6158763, -2.5403944, 0.0001616979, -36898.668, 1563.9761, 0.3025368, 12.949112, 0.94519504, -0.10529554, 0.42767234),
    14.0: (1.4599745, -2.4384563, 0.00013260597, -42999.894, 1951.246, 0.34871182, 7.6508471, 1.7045392, -0.12460128, -0.25382056),
}
# Combined slip (MF 6.1/6.2), lab surface; kappa = SAE slip ratio, alpha in rad, dfz = (Fz - 1000)/1000.
GXA_PARAMS = (16.169001, 12.802699, 1.0830827, 0.47719074, 0.55180544)    # rBx1, rBx2, rCx1, rEx1, rEx2
GYK_PARAMS = (17.47141, 16.874306, 0.99118381, -0.0003007915, -0.17909628, 0.0094717603, 0.016046406)
#             rBy1,     rBy2,      rCy1,       rEy1,           rEy2,        rHy1,          rHy2
FZ_NOM = 1000.0
DATA_MAX_FZ_LATERAL = 1264.0        # N, highest load in the 16x7.5-10 cornering data


def _shape(x, C, E):
    return math.sin(C * math.atan(x - E * (x - math.atan(x))))


@lru_cache(maxsize=64)
def _x_peak(C, E):
    """Where the lateral shape function peaks in x = B*alpha (depends only on C and E)."""
    xs = np.linspace(1e-4, 50.0, 20000)
    return float(xs[int(np.argmax(np.sin(C * np.arctan(xs - E * (xs - np.arctan(xs))))))])


def _weight(B, C, E, x):
    """MF combined-slip weighting term cos(C*atan(B*x - E*(B*x - atan(B*x))))."""
    bx = B * x
    return math.cos(C * math.atan(bx - E * (bx - math.atan(bx))))


class R20_V2_Tire_Model:
    TRACK_GRIP_SCALE = R20_V1_Tire_Model.TRACK_GRIP_SCALE
    # Also read by the dynamic model (Dynamics/solver/vehicle_model.py)
    GXA_PARAMS, GYK_PARAMS, FZ_NOM = GXA_PARAMS, GYK_PARAMS, FZ_NOM

    def __init__(self, wheel_radius=0.203, psi=12.0, falloff_slope=DEFAULT_FALLOFF_SLOPE, grip_scale=None):
        """grip_scale: None = TRACK_GRIP_SCALE (track, what the sims use); 1.0 = TTC lab surface."""
        self.wheel_radius = wheel_radius
        self.psi = psi
        self.falloff_slope = falloff_slope
        self.grip_scale = self.TRACK_GRIP_SCALE if grip_scale is None else float(grip_scale)
        long_cls = R20_V1_Tire_Model
        if self.grip_scale != R20_V1_Tire_Model.TRACK_GRIP_SCALE:
            long_cls = type("R20_V1_Tire_Model_scaled", (R20_V1_Tire_Model,), {"TRACK_GRIP_SCALE": self.grip_scale})
        self._long = long_cls(wheel_radius=wheel_radius, psi=psi, falloff_slope=falloff_slope)
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

    # ------------------------------------------------------------ same as the falloff model
    def tire_slip_ratio(self, wheel_omega, vx):
        return self._long.tire_slip_ratio(wheel_omega, vx)

    def peak_longitudinal(self, fz):
        return self._long.peak_longitudinal(fz)

    def peak_slip_ratio(self, fz):
        return self._long.peak_slip_ratio(fz)

    # ------------------------------------------------------------ lateral
    def _fy_pure(self, fz, alpha, g):
        """Pure lateral force (N); alpha, g (camber) in rad. Held at the curve's own peak past
        the peak slip angle, as R20_Legacy_Tire_Model does (data stops at ~12.5 deg lab)."""
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
        convention) and slip angle (rad). Slip is divided by the grip scale (see docstring)."""
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
        alpha gives negative Fy, as in R20_Legacy_Tire_Model); camber_deg: inclination angle (deg, SAE).
        With alpha = 0 and no camber this is exactly R20_V1_Tire_Model.tire_forces()."""
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
