from functools import lru_cache
import numpy as np

@lru_cache(maxsize=64)
def _find_x_peak(C, E, x_max, n=20000):
    """Where sin(C*atan(x - E*(x - atan(x)))) peaks, in dimensionless x=B*slip space -
    independent of Fz, so this only depends on (C, E). Cached at module level since
    car configs are commonly re-instantiated many times in a sweep with the same
    fitted (C, E) (e.g. Endurance.py's power_limit sweep constructs ~81 cars), and this
    search is too expensive to redo identically on every one of them."""
    xs = np.linspace(1e-4, x_max, n)
    ys = np.sin(C * np.arctan(xs - E * (xs - np.arctan(xs))))
    return float(xs[int(np.argmax(ys))])

class PacejkaR20TireConfig:
    model_name = "pacejka_r20"
    
    """Pure-slip Pacejka 'Magic Formula' tire model for the Hoosier R20 compound,
    fit directly to real Calspan TTC Round 9 (project 2356) raw run data - see
    build_dataset.py and fit_pacejka_by_pressure.py in this folder for the full
    pipeline.

    Unlike the Dugoff-style model in SR_17.py/SR_18.py (whose saturation function
    is monotonically non-decreasing in slip - it approaches tire_mu*Fz but never
    falls back off it), this model has a genuine peak followed by a falloff past
    peak slip, because it's fit directly to the shape the real tire data shows
    rather than assumed to only saturate.

    Data provenance and scope:
      - Both fits pool ALL R20-compound TTC runs from this round: 16x6.0-10,
        16x7.5-10 (the actual FSAE car's size), 18.0x6.0-10, and 20.5x7.0-13,
        all Hoosier 43070/43075/43100/43164 R20 C2000 - same rubber compound,
        different carcass size, per the team's request to use every R20 run to
        better-constrain the fit for the 16x7.5-10 tire specifically.
      - Hoosier ran NO drive/brake (slip ratio) tests at all on either 16" R20
        size in this TTC round - only cornering (slip angle) sweeps exist for
        16x6.0-10/16x7.5-10. So the longitudinal (Fx vs kappa) fit is entirely
        extrapolated from the 18x6.0-10 and 20.5x7.0-13 R20 data; only the
        lateral (Fy vs alpha) fit has any real 16" data behind it.
      - Both fits are PURE-slip only (alpha=0 for the longitudinal fit's input
        data, kappa~0 for the lateral fit's, exactly like how Dugoff's Cs/Ca
        were characterized). Combined-slip (kappa and alpha both nonzero) has
        no TTC sweep in this dataset to fit against, so it's handled by an
        approximate friction-circle scaling below, not an independent fit -
        nothing in this project currently drives both kappa and alpha nonzero
        at once (the Accel sims hold alpha=0 throughout; Skidpad.py's
        grip-circle check uses peak_lateral()/peak_longitudinal() below rather
        than calling tire_forces() with an actual slip ratio and angle).
      - No camber (IA) term is fit - the team's own prior TTC review already
        concluded this tire has no meaningful camber sensitivity (see
        SR_17.py's tires_camber_sensitive comment), so camber sweeps in the
        raw data were pooled in as ordinary points rather than fit separately.

    Pressure dependence: the TTC sweep tested 4 discrete pressure setpoints
    (~8, 10, 12, 14 psi - see the Run Guide's "Initial 12psi -> 10,14psi ->
    8psi -> Final 12psi" schedule), each fit separately below. A requested psi
    linearly interpolates the 6 Magic Formula coefficients between the two
    bracketing setpoints (clamped to [8, 14] psi - no data outside that range
    to extrapolate from), then TRACK_GRIP_SCALE is applied (see its definition
    below). Within the tested range, both peak Fx and peak Fy decrease
    monotonically as pressure increases in the raw fit - lower pressure means
    more mechanical grip here, consistent with typical slick-tire behavior in
    this operating range.

    Fitted form (pure slip, per axis, at a given pressure):
        D(Fz) = c1*Fz + c2*Fz^2                      (peak force vs load)
        K(Fz) = c3*Fz + c4*Fz^2                       (slip stiffness vs load)
        B(Fz) = K(Fz) / (C * D(Fz))
        F(slip, Fz) = D(Fz) * sin(C*atan(B(Fz)*slip - E*(B(Fz)*slip - atan(B(Fz)*slip))))
    kappa is the dimensionless slip ratio (same convention as SR_17/SR_18's
    tire_slip_ratio); alpha is the slip angle in radians.
    """

    # Per-pressure-setpoint (C, c1, c2, c3, c4, E), fit separately at each psi bin.
    # Longitudinal (Fx vs kappa): R^2 = 0.933 / 0.956 / 0.951 / 0.963 at 8/10/12/14 psi.
    LONG_PARAMS_BY_PSI = {
        8.0:  (1.59437457, 2.53820947, -1.01093142e-04, 72.3693722, -2.08747316e-02, 0.550908782),
        10.0: (1.70610545, 2.42349229, -4.79777938e-05, 54.8157498, -9.35985366e-03, 1.24475595),
        12.0: (2.04476080, 2.41180174, -1.48005654e-04, 39.4564538, 4.62119926e-03, 1.40326069),
        14.0: (2.31789805, 2.10395882, -3.38556844e-05, 31.2779074, 4.75644462e-03, 1.66649286),
    }
    # Lateral (Fy vs alpha[rad]): R^2 = 0.981 / 0.988 / 0.991 / 0.993 at 8/10/12/14 psi.
    LAT_PARAMS_BY_PSI = {
        8.0:  (1.50947913, -2.83657149, 4.51756301e-04, -49.5286325, 1.59921706e-02, 0.394626256),
        10.0: (1.59612561, -2.67151215, 2.95026657e-04, -46.2834831, 1.00597590e-02, 0.401260100),
        12.0: (1.54391850, -2.57930235, 2.68870656e-04, -40.0029126, 4.03948147e-03, 0.395946215),
        14.0: (1.47185448, -2.47152140, 1.96143492e-04, -35.6183079, 9.10428443e-04, 0.392662828),
    }
    PSI_MIN = min(LONG_PARAMS_BY_PSI)
    PSI_MAX = max(LONG_PARAMS_BY_PSI)

    # Track-grip calibration: TTC runs are done on Calspan's standardized lab test
    # surface (Test Schedule's Road Condition column: "120, 3-Mite, Stoned" - a
    # conditioned abrasive surface, not asphalt), which reads higher peak grip than a
    # real track. Raw-fit peak forces gave SR_17 a 49.0s Endurance lap in sim against a
    # real reference of ~59s for the same car/track - about 60% more grip than actually
    # achievable. This factor scales down only the peak-force terms (c1, c2 - the D(Fz)
    # amplitude), leaving the stiffness terms (c3, c4) untouched: initial force-per-degree
    # response near zero slip is mostly a carcass/tread-block property, largely
    # surface-independent, while peak force is limited by available friction at the
    # contact patch, which does depend on the surface - so a "same tire, lower-grip real
    # surface" correction should lower the peak without changing the initial slope.
    # 0.65 was found by sweeping this factor against Endurance.py's SR_17 lap time until
    # it matched the ~59s real reference (0.65 -> 59.07s simulated).
    TRACK_GRIP_SCALE = 0.65

    # The classic Pacejka sin(C*atan(x - E*(x - atan(x)))) form is unbounded, and past
    # its true peak it curves back down - extrapolated far enough, it crosses back
    # through zero and goes *negative* (a real risk here: this project's accel sims have
    # already produced transient wheelspin above 70% slip). The raw TTC sweeps only
    # actually cover kappa in about [-0.30, +0.23] and alpha in about [-12.5, +12.6] deg,
    # so nothing past that range is supported by data anyway. Rather than a fixed slip
    # boundary (which broke once TRACK_GRIP_SCALE shifted the peak-force terms without
    # moving the stiffness terms, pulling the true peak-slip location away from wherever
    # a hardcoded boundary happened to be tuned for), the peak location in the
    # dimensionless x=B*slip space is found once per axis at construction time (it
    # depends only on C and E, not on Fz or slip) and slip is clamped so B*slip never
    # exceeds it - the force plateaus at the curve's own true peak beyond that, for any
    # combination of psi/TRACK_GRIP_SCALE/Fz.
    #
    # This was briefly replaced with a genuine falloff (down to a floor part-way to
    # the curve's own zero-crossing) so a launch/traction controller's target slip
    # ratio setting would actually matter past the peak-slip point - it does give a
    # more realistic single-axle result (e.g. a real optimum in SR_17's target slip
    # ratio sweep instead of a flat plateau), but on SR_18's psi=12 fit (whose curve
    # does cross zero) it made Acceleration_AWD.py's uncontrolled 4-motor launch (no
    # traction control at all) noticeably less accurate - small, sensitive parameter
    # changes could tip the coupled front/rear wheelspin dynamics into a qualitatively
    # different trajectory, and required floor-fraction tuning has no measured data
    # to anchor it. Reverted back to the flat peak-hold: the RWD single-axle sweeps
    # go back to plateauing past peak slip, but the AWD results (times and both sweep
    # graphs) are more trustworthy, which matters more right now.
    X_PEAK_SEARCH_MAX = 50.0
    

    def __init__(self, radius_m=0.2032, psi=12.0, track_grip_scale=0.65):
        self.radius_m = radius_m
        self.psi = psi
        self.track_grip_scale = track_grip_scale
        self.long_params = self._interp_params(psi, self.LONG_PARAMS_BY_PSI, self.track_grip_scale)
        self.lat_params = self._interp_params(psi, self.LAT_PARAMS_BY_PSI, self.track_grip_scale)
        self._x_peak_long = _find_x_peak(self.long_params[0], self.long_params[5], self.X_PEAK_SEARCH_MAX)
        self._x_peak_lat = _find_x_peak(self.lat_params[0], self.lat_params[5], self.X_PEAK_SEARCH_MAX)

    @classmethod
    def _interp_params(cls, psi, params_by_psi, track_grip_scale):
        """Linearly interpolates all 6 coefficients between the two bracketing tested
        pressure setpoints, clamping psi to [PSI_MIN, PSI_MAX] - no data outside that
        range to extrapolate from. Then applies TRACK_GRIP_SCALE to the peak-force
        terms (c1, c2) only - see its definition above."""
        psi_clamped = max(cls.PSI_MIN, min(cls.PSI_MAX, psi))
        psis = sorted(params_by_psi)
        if psi_clamped in params_by_psi:
            C, c1, c2, c3, c4, E = params_by_psi[psi_clamped]
        else:
            lo = max(p for p in psis if p <= psi_clamped)
            hi = min(p for p in psis if p >= psi_clamped)
            t = (psi_clamped - lo) / (hi - lo)
            lo_params = np.array(params_by_psi[lo])
            hi_params = np.array(params_by_psi[hi])
            C, c1, c2, c3, c4, E = lo_params + t * (hi_params - lo_params)
        return (C, c1 * track_grip_scale, c2 * track_grip_scale, c3, c4, E)

    @staticmethod
    def _shape(x, C, E):
        return np.sin(C * np.arctan(x - E * (x - np.arctan(x))))

    def _pure(self, slip, fz, params, x_peak):
        C, c1, c2, c3, c4, E = params
        D = c1 * fz + c2 * fz * fz
        K = c3 * fz + c4 * fz * fz
        D_safe = np.sign(D) * max(abs(D), 1.0) if D != 0 else 1.0
        B = K / (C * D_safe)
        Bs = max(-x_peak, min(x_peak, B * slip))
        return D * self._shape(Bs, C, E)

    def _peak(self, fz, params, x_peak):
        C, c1, c2, _, _, E = params
        D = c1 * fz + c2 * fz * fz
        return D * self._shape(x_peak, C, E)

    def peak_longitudinal(self, fz):
        """Peak achievable |Fx| (N) at this normal load fz (N), at this instance's
        configured psi - the true peak of the fitted longitudinal curve (D(Fz) scaled
        by the shape function's own maximum, not just the raw D(Fz) amplitude term).
        Public so callers that need a grip budget without driving an actual slip ratio
        through tire_forces() (e.g. Skidpad.py's steady-state feasibility check) can
        use the real load-dependent peak instead of a flat tire_mu*Fz."""
        if fz <= 0.0:
            return 0.0
        return abs(self._peak(fz, self.long_params, self._x_peak_long))

    def peak_lateral(self, fz):
        """Peak achievable |Fy| (N) at this normal load fz (N), at this instance's
        configured psi - see peak_longitudinal. Returned as a magnitude (the fitted
        D(Fz) here is negative, by the SAE sign convention the raw TTC data uses for
        Fy vs positive slip angle)."""
        if fz <= 0.0:
            return 0.0
        return abs(self._peak(fz, self.lat_params, self._x_peak_lat))

    def tire_forces(self, fz, kappa, alpha=0.0):
        """Returns (Fx, Fy) in N for normal load fz (N), slip ratio kappa, and
        slip angle alpha (radians), at this instance's configured psi. Pure-slip
        cases (alpha==0 or kappa==0) use the exact fitted Pacejka curve for that
        axis (held flat at its own true peak past the peak-slip point - see
        X_PEAK_SEARCH_MAX above); true combined slip (both nonzero) is approximated
        by scaling both pure-slip values down to stay within a combined friction
        circle sized by each axis's own fitted peak force at this Fz - see the class
        docstring's combined-slip caveat.
        """
        if fz <= 0.0:
            return 0.0, 0.0

        fx0 = self._pure(kappa, fz, self.long_params, self._x_peak_long)
        fy0 = self._pure(alpha, fz, self.lat_params, self._x_peak_lat)

        dx = self._peak(fz, self.long_params, self._x_peak_long)
        dy = self._peak(fz, self.lat_params, self._x_peak_lat)
        ex = fx0 / dx if dx != 0.0 else 0.0
        ey = fy0 / dy if dy != 0.0 else 0.0
        resultant = np.hypot(ex, ey)

        if resultant > 1.0:
            scale = 1.0 / resultant
            fx0 *= scale
            fy0 *= scale

        return fx0, fy0

    def calculateForces(self, Fz_N, slip_ratio, slip_angle_rad):
        Fx, Fy = self.tire_forces(Fz_N, slip_ratio, slip_angle_rad)
        return float(Fx), -float(Fy)    # TTC data weird flipping so positive slip angle gives positive (left) Fy

    def getPeakLateral_N(self, Fz_N):
        return float(self.peak_lateral(Fz_N))

    def getPeakLongitudinal_N(self, Fz_N):
        return float(self.peak_longitudinal(Fz_N))

    def isAtGripLimit(self):
        return self.normalized_slip >= 1.0 - 1e-9


