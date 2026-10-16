"""Tests for the Magic Formula tire in Vehicle/Tire/MagicFormulaTireConfig.py.

NOTE: These test cases are AI-generated on purpose, as an independent check on the hand-written model.

Reference (written out again below, independent of the class):
    Fx0 = D sin(C atan(B k - E (B k - atan(B k)))),  D = mu Fz,  B = c_long / (C mu)   (so B C D = c_long Fz)
    combined slip: kn = k / k_peak, an = a / a_peak, rho = hypot(kn, an), Fx = kn / rho * Fx0(rho k_peak)
"""
import math

import pytest

from Vehicle.Tire.MagicFormulaTireConfig import MagicFormulaTireConfig
from Vehicle.Tire.TireConfig import TireConfig

MU = 1.5
C_LONG, E_LONG, C_LONG_PER_LOAD = 1.5, 0.0, 25.691
C_LAT, E_LAT, C_LAT_PER_LOAD = 1.3, 0.0, 33.959
FZ = 667.0


def make_tire(**overrides):
    kwargs = dict(radius_m=0.2032, mu=MU, C_long=C_LONG, E_long=E_LONG, long_stiff_per_load=C_LONG_PER_LOAD,
                  C_lat=C_LAT, E_lat=E_LAT, lat_stiff_per_load=C_LAT_PER_LOAD)
    kwargs.update(overrides)
    return MagicFormulaTireConfig(**kwargs)


def magic_formula(slip, fz, stiff_per_load, C, E, mu=MU):
    B = stiff_per_load / (C * mu)
    return mu * fz * math.sin(C * math.atan(B * slip - E * (B * slip - math.atan(B * slip))))


class TestConstructorValidation:
    @pytest.mark.parametrize("bad", [
        dict(radius_m=0.0), dict(radius_m=-0.2), dict(mu=-1.0),
        dict(C_long=1.0), dict(C_long=2.0), dict(C_lat=0.5),
        dict(E_long=1.1), dict(E_lat=2.0),
        dict(long_stiff_per_load=0.0), dict(lat_stiff_per_load=-3.0),
    ])
    def test_bad_values_raise(self, bad):
        with pytest.raises(ValueError):
            make_tire(**bad)

    def test_negative_E_is_allowed(self):
        make_tire(E_long=-0.5, E_lat=-1.0)


class TestPureLongitudinal:
    @pytest.mark.parametrize("k", [-1.0, -0.4, -0.1, -0.01, 0.0, 0.01, 0.05, 0.2, 0.7, 1.0])
    def test_matches_the_formula(self, k):
        fx, fy = make_tire().calculateForces(FZ, k, 0.0)
        assert fx == pytest.approx(magic_formula(k, FZ, C_LONG_PER_LOAD, C_LONG, E_LONG), abs=1e-9)
        assert fy == 0.0

    def test_small_slip_stiffness_matches_dugoff_tire1(self):
        dugoff = TireConfig(radius_m=0.2032, longitudinal_stiffness_per_load=C_LONG_PER_LOAD,
                            lateral_stiffness_per_load=C_LAT_PER_LOAD, tireroadfriction=MU)
        fx_mf, _ = make_tire().calculateForces(FZ, 0.001, 0.0)
        fx_dugoff, _ = dugoff.calculateForces(FZ, 0.001, 0.0)
        assert fx_mf == pytest.approx(C_LONG_PER_LOAD * FZ * 0.001, rel=1e-3)
        assert fx_mf == pytest.approx(fx_dugoff, rel=1e-3)

    def test_peak_is_mu_fz_at_the_peak_slip(self):
        tire = make_tire()
        # with E = 0 the peak is at k = tan(pi / (2C)) / B
        assert tire.peak_slip_ratio == pytest.approx(math.tan(math.pi / (2 * C_LONG)) / (C_LONG_PER_LOAD / (C_LONG * MU)))
        fx, _ = tire.calculateForces(FZ, tire.peak_slip_ratio, 0.0)
        assert fx == pytest.approx(MU * FZ, rel=1e-9)
        for k in (tire.peak_slip_ratio * 0.9, tire.peak_slip_ratio * 1.1):
            assert tire.calculateForces(FZ, k, 0.0)[0] < fx

    def test_peak_slip_with_nonzero_E_is_still_the_peak(self):
        tire = make_tire(E_long=-0.8)
        peak = tire.calculateForces(FZ, tire.peak_slip_ratio, 0.0)[0]
        assert peak == pytest.approx(MU * FZ, rel=1e-9)

    def test_odd_in_slip(self):
        tire = make_tire()
        assert tire.calculateForces(FZ, -0.2, 0.0)[0] == pytest.approx(-tire.calculateForces(FZ, 0.2, 0.0)[0])

    def test_drops_past_the_peak_toward_the_sliding_value(self):
        fx_spin, _ = make_tire().calculateForces(FZ, 50.0, 0.0)
        assert fx_spin == pytest.approx(MU * FZ * math.sin(C_LONG * math.pi / 2), rel=1e-2)

    def test_force_scales_with_load(self):
        tire = make_tire()
        assert tire.calculateForces(2 * FZ, 0.1, 0.0)[0] == pytest.approx(2 * tire.calculateForces(FZ, 0.1, 0.0)[0])

    @pytest.mark.filterwarnings("error")
    def test_no_load_or_no_slip_means_no_force(self):
        assert make_tire().calculateForces(0.0, 0.2, 0.1) == (0.0, 0.0)
        assert make_tire().calculateForces(FZ, 0.0, 0.0) == (0.0, 0.0)


class TestPureLateral:
    @pytest.mark.parametrize("a", [-0.5, -0.1, 0.0, 0.02, 0.1, 0.3])
    def test_matches_the_formula(self, a):
        fx, fy = make_tire().calculateForces(FZ, 0.0, a)
        assert fy == pytest.approx(magic_formula(a, FZ, C_LAT_PER_LOAD, C_LAT, E_LAT), abs=1e-9)
        assert fx == pytest.approx(0.0, abs=1e-12)

    def test_has_the_sign_of_the_slip_angle_like_dugoff(self):
        assert make_tire().calculateForces(FZ, 0.0, 0.05)[1] > 0
        assert make_tire().calculateForces(FZ, 0.0, -0.05)[1] < 0


class TestCombinedSlip:
    def test_slip_angle_takes_away_longitudinal_force(self):
        tire = make_tire()
        fx_pure, _ = tire.calculateForces(FZ, 0.1, 0.0)
        fx_combined, fy_combined = tire.calculateForces(FZ, 0.1, 0.1)
        assert 0 < fx_combined < fx_pure
        assert fy_combined > 0

    def test_slip_ratio_takes_away_lateral_force(self):
        tire = make_tire()
        _, fy_pure = tire.calculateForces(FZ, 0.0, 0.1)
        _, fy_combined = tire.calculateForces(FZ, 0.1, 0.1)
        assert 0 < fy_combined < fy_pure

    def test_never_more_than_mu_fz(self):
        tire = make_tire()
        for k in (-1.0, -0.5, -0.1, 0.0, 0.05, 0.15, 0.5, 3.0):
            for a in (-0.6, -0.15, 0.0, 0.05, 0.15, 0.6):
                fx, fy = tire.calculateForces(FZ, k, a)
                assert math.hypot(fx, fy) <= MU * FZ * (1 + 1e-9)


class TestGripLimitAndState:
    def test_grip_limit_is_at_the_peak(self):
        tire = make_tire()
        tire.calculateForces(FZ, 0.9 * tire.peak_slip_ratio, 0.0)
        assert not tire.isAtGripLimit()
        tire.calculateForces(FZ, tire.peak_slip_ratio, 0.0)
        assert tire.isAtGripLimit()
        tire.calculateForces(0.0, 1.0, 0.0)
        assert not tire.isAtGripLimit()   # no load, nothing to be at the limit of

    def test_slip_helpers_match_dugoff(self):
        tire = make_tire()
        assert tire.calculateTireSlipRatio(11.0, 10.0) == pytest.approx(0.1)
        assert tire.calculateTireSlipRatio(5.0, 0.0) == 0.0
        assert tire.calculateTireSlipAngle(0.05, 15.0, 1.5) == pytest.approx(0.05 - math.atan(0.1))

    def test_two_tires_do_not_share_state(self):
        a, b = make_tire(), make_tire()
        a.calculateForces(FZ, 0.3, 0.1)
        assert b.getTireSlipRatio() == 0.0 and b.getTireSlipAngle() == 0.0
