"""Tests for the Dugoff tire model in Vehicle/Tire/TireConfig.py.

NOTE: These test cases are AI-generated on purpose, as an independent check on the hand-written model.

Call-order contract these tests assume (change `solve()` below in ONE place if your order changes):
    tire.calculateTireSlipRatio(wheel_speed, vehicle_speed)
    tire.calculateTireSlipAngle(wheel_angle, vx, vy)
    lam = tire.calculateCoupling(Fz)            # dimensionless lambda
    Fx  = tire.calculateTireLongitudinalForce() # saturation already applied
    Fy  = tire.calculateTireLateralForce()      # saturation already applied

Conventions:
    sigma = (wheel_speed - vehicle_speed) / vehicle_speed     driving > 0, braking < 0, locked = -1
    alpha = wheel_angle - atan(vy / vx)
    Fx > 0 pushes the car forward. Fy opposes lateral sliding (vy > 0, straight wheel -> Fy < 0).

Textbook Dugoff (reference implementation below):
    lambda = mu*Fz*(1+sigma) / (2*sqrt((Cs*sigma)^2 + (Ca*tan(alpha))^2))
    f      = lambda*(2-lambda) if lambda < 1 else 1
    Fx     = Cs*sigma/(1+sigma) * f
    Fy     = Ca*tan(alpha)/(1+sigma) * f
"""
import math

import pytest

from Vehicle.Tire.TireConfig import TireConfig

CS = 50000.0   # longitudinal stiffness, N per unit slip
CA = 60000.0   # lateral stiffness, N/rad
MU = 1.5
RADIUS = 0.2286
FZ = 800.0     # N, so the friction limit mu*Fz = 1200 N

V = 10.0       # vehicle speed used to build wheel speeds
VX = 15.0      # longitudinal velocity used to build slip angles


def make_tire(**overrides):
    kwargs = dict(radius_m=RADIUS, longitudinal_stiffness=CS, lateral_stiffness=CA, tireroadfriction=MU)
    kwargs.update(overrides)
    return TireConfig(**kwargs)


def solve(tire, sigma, alpha, fz):
    """Drive a tire to (sigma, alpha, Fz) and return (Fx, Fy, lambda)."""
    tire.calculateTireSlipRatio(V * (1 + sigma), V)
    tire.calculateTireSlipAngle(alpha, VX, 0.0)
    lam = tire.calculateCoupling(fz)
    fx = tire.calculateTireLongitudinalForce()
    fy = tire.calculateTireLateralForce()
    return fx, fy, lam


def dugoff(sigma, alpha, fz, cs=CS, ca=CA, mu=MU):
    """Independent textbook implementation. Returns (Fx, Fy, lambda)."""
    fx_lin = cs * sigma
    fy_lin = ca * math.tan(alpha)
    mag = math.hypot(fx_lin, fy_lin)
    if mag == 0.0:
        return 0.0, 0.0, math.inf
    if sigma == -1.0:  # locked wheel: limit of the formula (lambda -> 0), full sliding friction
        return mu * fz * fx_lin / mag, mu * fz * fy_lin / mag, 0.0
    lam = mu * fz * (1 + sigma) / (2 * mag)
    f = 1.0 if lam >= 1 else lam * (2 - lam)
    return fx_lin / (1 + sigma) * f, fy_lin / (1 + sigma) * f, lam


# ---------------------------------------------------------------- constructor validation
class TestConstructorValidation:
    @pytest.mark.parametrize(
        "bad",
        [
            dict(radius_m=0.0),
            dict(radius_m=-0.2),
            dict(longitudinal_stiffness=-1.0),
            dict(lateral_stiffness=-1.0),
            dict(tireroadfriction=-0.5),
        ],
    )
    def test_negative_or_zero_radius_and_negative_params_raise(self, bad):
        with pytest.raises(ValueError):
            make_tire(**bad)

    @pytest.mark.parametrize(
        "bad",
        [
            dict(longitudinal_stiffness=None, longitudinal_stiffness_per_load=-1.0),
            dict(lateral_stiffness=None, lateral_stiffness_per_load=-1.0),
            dict(longitudinal_stiffness_per_load=25.0),  # fixed CS is also set by make_tire -> both
            dict(lateral_stiffness_per_load=30.0),       # fixed CA is also set by make_tire -> both
        ],
    )
    def test_bad_per_load_stiffness_raises(self, bad):
        with pytest.raises(ValueError):
            make_tire(**bad)

    def test_valid_tire_constructs(self):
        tire = make_tire()
        assert tire.radius_m == RADIUS
        assert tire.longitudinal_stiffness == CS
        assert tire.lateral_stiffness == CA
        assert tire.tireroadfriction == MU


# ---------------------------------------------------------------- slip helpers
class TestSlipRatio:
    @pytest.mark.parametrize(
        "wheel, vehicle, expected",
        [(11.0, 10.0, 0.1), (9.0, 10.0, -0.1), (10.0, 10.0, 0.0), (0.0, 10.0, -1.0), (20.0, 10.0, 1.0)],
    )
    def test_values(self, wheel, vehicle, expected):
        assert make_tire().calculateTireSlipRatio(wheel, vehicle) == pytest.approx(expected)

    @pytest.mark.parametrize("wheel, vehicle", [(11.0, 10.0), (0.0, 10.0), (5.0, 0.0), (0.0, 0.0)])
    def test_return_value_matches_stored_state(self, wheel, vehicle):
        tire = make_tire()
        returned = tire.calculateTireSlipRatio(wheel, vehicle)
        assert tire.getTireSlipRatio() == returned
        assert math.isfinite(returned)

    def test_standstill_call_does_not_leave_stale_value(self):
        tire = make_tire()
        tire.calculateTireSlipRatio(11.0, 10.0)          # sigma = 0.1
        returned = tire.calculateTireSlipRatio(5.0, 0.0)  # Vx = 0: sigma undefined
        assert tire.getTireSlipRatio() == returned
        assert tire.getTireSlipRatio() != pytest.approx(0.1)


class TestSlipAngle:
    @pytest.mark.parametrize(
        "delta, vx, vy, expected",
        [
            (0.05, 15.0, 1.5, 0.05 - math.atan(0.1)),
            (0.0, 15.0, 0.0, 0.0),
            (0.0, 15.0, 1.5, -math.atan(0.1)),
            (0.1, 10.0, 0.0, 0.1),
            (-0.1, 10.0, 0.0, -0.1),
        ],
    )
    def test_values(self, delta, vx, vy, expected):
        assert make_tire().calculateTireSlipAngle(delta, vx, vy) == pytest.approx(expected)

    @pytest.mark.parametrize("delta, vx, vy", [(0.1, 15.0, 1.0), (0.1, 0.0, 1.0), (0.0, 0.0, 0.0)])
    def test_return_value_matches_stored_state(self, delta, vx, vy):
        tire = make_tire()
        returned = tire.calculateTireSlipAngle(delta, vx, vy)
        assert tire.getTireSlipAngle() == returned
        assert math.isfinite(returned)

    def test_standstill_call_does_not_leave_stale_value(self):
        tire = make_tire()
        tire.calculateTireSlipAngle(0.1, 15.0, 1.0)
        returned = tire.calculateTireSlipAngle(0.1, 0.0, 1.0)
        assert tire.getTireSlipAngle() == returned


# ---------------------------------------------------------------- force signs
class TestForceSigns:
    def test_driving_wheel_pushes_car_forward(self):
        fx, _, _ = solve(make_tire(), sigma=0.1, alpha=0.0, fz=FZ)
        assert fx > 0

    def test_braking_wheel_pushes_car_backward(self):
        fx, _, _ = solve(make_tire(), sigma=-0.1, alpha=0.0, fz=FZ)
        assert fx < 0

    def test_lateral_force_opposes_sideways_slide(self):
        tire = make_tire()
        tire.calculateTireSlipRatio(10.0, 10.0)
        tire.calculateTireSlipAngle(0.0, 15.0, 1.5)  # car sliding toward +y, wheel straight
        tire.calculateCoupling(FZ)
        tire.calculateTireLongitudinalForce()
        assert tire.calculateTireLateralForce() < 0

    def test_lateral_force_odd_in_slip_angle(self):
        _, fy_pos, _ = solve(make_tire(), 0.05, 0.03, FZ)
        _, fy_neg, _ = solve(make_tire(), 0.05, -0.03, FZ)
        assert fy_pos == pytest.approx(-fy_neg)

    def test_longitudinal_force_even_in_slip_angle(self):
        fx_pos, _, _ = solve(make_tire(), 0.05, 0.03, FZ)
        fx_neg, _, _ = solve(make_tire(), 0.05, -0.03, FZ)
        assert fx_pos == pytest.approx(fx_neg)


# ---------------------------------------------------------------- agrees with textbook Dugoff
SIGMAS = [-0.5, -0.1, 0.0, 0.02, 0.1, 0.5, 2.0]
ALPHAS = [-0.3, -0.05, 0.0, 0.01, 0.05, 0.2, 0.6]
FZS = [300.0, 800.0, 1500.0]


class TestAgainstTextbook:
    @pytest.mark.parametrize("fz", FZS)
    @pytest.mark.parametrize("alpha", ALPHAS)
    @pytest.mark.parametrize("sigma", SIGMAS)
    def test_forces_match_reference(self, sigma, alpha, fz):
        fx, fy, _ = solve(make_tire(), sigma, alpha, fz)
        ref_fx, ref_fy, _ = dugoff(sigma, alpha, fz)
        assert fx == pytest.approx(ref_fx, rel=1e-6, abs=1e-6)
        assert fy == pytest.approx(ref_fy, rel=1e-6, abs=1e-6)

    @pytest.mark.parametrize("fz", FZS)
    @pytest.mark.parametrize("alpha", ALPHAS)
    @pytest.mark.parametrize("sigma", SIGMAS)
    def test_lambda_matches_reference(self, sigma, alpha, fz):
        if sigma == 0.0 and alpha == 0.0:
            pytest.skip("lambda is infinite at zero slip; covered by TestEdgeCases")
        _, _, lam = solve(make_tire(), sigma, alpha, fz)
        _, _, ref_lam = dugoff(sigma, alpha, fz)
        assert lam == pytest.approx(ref_lam, rel=1e-6)

    def test_different_parameters_change_the_answer(self):
        tire = make_tire(longitudinal_stiffness=80000.0, lateral_stiffness=30000.0, tireroadfriction=1.1)
        fx, fy, _ = solve(tire, 0.03, 0.04, 1000.0)
        ref_fx, ref_fy, _ = dugoff(0.03, 0.04, 1000.0, cs=80000.0, ca=30000.0, mu=1.1)
        assert fx == pytest.approx(ref_fx, rel=1e-6)
        assert fy == pytest.approx(ref_fy, rel=1e-6)


# ---------------------------------------------------------------- load-scaled stiffness (Cs = c_long * Fz)
C_LONG = 25.691
C_LAT = 33.959


def make_load_scaled_tire():
    return make_tire(
        longitudinal_stiffness=None,
        lateral_stiffness=None,
        longitudinal_stiffness_per_load=C_LONG,
        lateral_stiffness_per_load=C_LAT,
    )


class TestLoadScaledStiffness:
    @pytest.mark.parametrize("fz", FZS)
    @pytest.mark.parametrize("alpha", [-0.2, 0.0, 0.03, 0.4])
    @pytest.mark.parametrize("sigma", [-0.5, 0.0, 0.05, 0.5])
    def test_matches_reference_with_stiffness_times_load(self, sigma, alpha, fz):
        fx, fy, _ = solve(make_load_scaled_tire(), sigma, alpha, fz)
        ref_fx, ref_fy, _ = dugoff(sigma, alpha, fz, cs=C_LONG * fz, ca=C_LAT * fz)
        assert fx == pytest.approx(ref_fx, rel=1e-6, abs=1e-6)
        assert fy == pytest.approx(ref_fy, rel=1e-6, abs=1e-6)

    def test_small_slip_force_scales_with_load(self):
        fx_1, fy_1, _ = solve(make_load_scaled_tire(), 0.001, 0.001, FZ)
        fx_2, fy_2, _ = solve(make_load_scaled_tire(), 0.001, 0.001, 2 * FZ)
        assert fx_2 == pytest.approx(2 * fx_1, rel=1e-6)
        assert fy_2 == pytest.approx(2 * fy_1, rel=1e-6)

    def test_locked_wheel_uses_full_friction(self):
        fx, fy, _ = solve(make_load_scaled_tire(), -1.0, 0.1, FZ)
        assert math.hypot(fx, fy) == pytest.approx(MU * FZ, rel=1e-6)

    @pytest.mark.filterwarnings("error")
    def test_no_load_means_no_force(self):
        fx, fy, _ = solve(make_load_scaled_tire(), 0.2, 0.1, 0.0)
        assert fx == 0.0 and fy == 0.0


# ---------------------------------------------------------------- linear region
class TestLinearRegion:
    def test_small_slip_is_linear_in_stiffness(self):
        sigma, alpha = 0.001, 0.001
        fx, fy, lam = solve(make_tire(), sigma, alpha, fz=5000.0)  # huge load -> lambda >> 1
        assert lam > 1
        assert fx == pytest.approx(CS * sigma / (1 + sigma), rel=1e-6)
        assert fy == pytest.approx(CA * math.tan(alpha) / (1 + sigma), rel=1e-6)


# ---------------------------------------------------------------- saturation / friction limit
class TestSaturation:
    def test_total_force_never_exceeds_friction_limit(self):
        worst = 0.0
        for fz in (100.0, 800.0, 3000.0):
            for sigma in [-1.0, -0.9, -0.3, 0.0, 0.05, 0.3, 1.0, 3.0, 10.0]:
                for alpha in [-1.2, -0.4, -0.05, 0.0, 0.02, 0.1, 0.5, 1.2]:
                    fx, fy, _ = solve(make_tire(), sigma, alpha, fz)
                    assert math.isfinite(fx) and math.isfinite(fy), (sigma, alpha, fz)
                    assert math.hypot(fx, fy) <= MU * fz * (1 + 1e-9), (sigma, alpha, fz, fx, fy)
                    worst = max(worst, math.hypot(fx, fy) / (MU * fz))
        assert worst > 0.9  # sanity: the sweep really does reach the limit

    def test_pure_longitudinal_force_saturates_at_mu_fz(self):
        fx, fy, _ = solve(make_tire(), sigma=1000.0, alpha=0.0, fz=FZ)
        assert fx == pytest.approx(MU * FZ, rel=1e-2)
        assert fy == pytest.approx(0.0, abs=1e-6)

    def test_pure_lateral_force_saturates_at_mu_fz(self):
        _, fy, _ = solve(make_tire(), sigma=0.0, alpha=1.3, fz=FZ)
        assert fy == pytest.approx(MU * FZ, rel=1e-2)

    def test_lateral_force_is_nondecreasing_in_slip_angle(self):
        alphas = [i * 0.01 for i in range(0, 121)]
        fys = [solve(make_tire(), 0.0, a, FZ)[1] for a in alphas]
        assert all(b >= a - 1e-9 for a, b in zip(fys, fys[1:]))

    def test_no_kink_where_lambda_crosses_one(self):
        alpha_star = math.atan(MU * FZ / (2 * CA))  # sigma = 0: lambda == 1 here
        eps = 1e-7
        below = solve(make_tire(), 0.0, alpha_star - eps, FZ)[1]
        above = solve(make_tire(), 0.0, alpha_star + eps, FZ)[1]
        assert below == pytest.approx(above, rel=1e-4)

    def test_lateral_slip_reduces_available_longitudinal_force(self):
        fx_pure, _, _ = solve(make_tire(), 0.15, 0.0, FZ)
        fx_combined, _, _ = solve(make_tire(), 0.15, 0.15, FZ)
        assert 0 < fx_combined < fx_pure

    def test_longitudinal_slip_reduces_available_lateral_force(self):
        _, fy_pure, _ = solve(make_tire(), 0.0, 0.1, FZ)
        _, fy_combined, _ = solve(make_tire(), 0.5, 0.1, FZ)
        assert 0 < abs(fy_combined) < abs(fy_pure)

    def test_force_scales_with_normal_load_when_saturated(self):
        _, fy_1, _ = solve(make_tire(), 0.0, 1.3, FZ)
        _, fy_2, _ = solve(make_tire(), 0.0, 1.3, 2 * FZ)
        assert fy_2 == pytest.approx(2 * fy_1, rel=1e-2)

    def test_no_load_means_no_force(self):
        fx, fy, _ = solve(make_tire(), 0.2, 0.1, fz=0.0)
        assert fx == 0.0 and fy == 0.0


# ---------------------------------------------------------------- locked wheel (sigma = -1)
class TestLockedWheel:
    def test_exact_lock_straight_gives_full_braking_friction(self):
        fx, fy, _ = solve(make_tire(), sigma=-1.0, alpha=0.0, fz=FZ)
        assert fx == pytest.approx(-MU * FZ, rel=1e-6)
        assert fy == pytest.approx(0.0, abs=1e-6)

    def test_just_before_lock_is_close_to_full_braking_friction(self):
        fx, _, _ = solve(make_tire(), sigma=-0.999999, alpha=0.0, fz=FZ)
        assert fx == pytest.approx(-MU * FZ, rel=1e-3)

    @pytest.mark.parametrize("alpha", [-0.2, -0.02, 0.02, 0.2])
    def test_exact_lock_with_slip_angle_uses_full_friction_and_stays_finite(self, alpha):
        fx, fy, _ = solve(make_tire(), sigma=-1.0, alpha=alpha, fz=FZ)
        assert math.isfinite(fx) and math.isfinite(fy)
        assert math.hypot(fx, fy) == pytest.approx(MU * FZ, rel=1e-6)
        assert fx < 0
        assert fy * math.tan(alpha) > 0  # same sign as the unlocked lateral force

    def test_exact_lock_matches_reference(self):
        fx, fy, _ = solve(make_tire(), sigma=-1.0, alpha=0.1, fz=FZ)
        ref_fx, ref_fy, _ = dugoff(-1.0, 0.1, FZ)
        assert fx == pytest.approx(ref_fx, rel=1e-6)
        assert fy == pytest.approx(ref_fy, rel=1e-6)


# ---------------------------------------------------------------- edge cases / state
class TestEdgeCases:
    @pytest.mark.filterwarnings("error")
    def test_zero_slip_gives_zero_force_without_warnings_or_nan(self):
        fx, fy, lam = solve(make_tire(), 0.0, 0.0, FZ)
        assert fx == 0.0 and fy == 0.0
        assert not math.isnan(lam)

    @pytest.mark.filterwarnings("error")
    def test_pure_longitudinal_and_pure_lateral_have_no_warnings(self):
        solve(make_tire(), 0.1, 0.0, FZ)
        solve(make_tire(), 0.0, 0.1, FZ)

    def test_force_methods_are_idempotent(self):
        tire = make_tire()
        fx1, fy1, _ = solve(tire, 0.2, 0.1, FZ)
        fx2 = tire.calculateTireLongitudinalForce()
        fy2 = tire.calculateTireLateralForce()
        fx3 = tire.calculateTireLongitudinalForce()
        assert fx1 == pytest.approx(fx2) == pytest.approx(fx3)
        assert fy1 == pytest.approx(fy2)

    def test_repeated_solve_gives_same_answer(self):
        tire = make_tire()
        first = solve(tire, 0.2, 0.1, FZ)
        solve(tire, -0.4, -0.3, 300.0)  # disturb the state
        assert solve(tire, 0.2, 0.1, FZ) == pytest.approx(first)

    def test_two_tires_do_not_share_state(self):
        a, b = make_tire(), make_tire()
        a.calculateTireSlipRatio(15.0, 10.0)
        b.calculateTireSlipRatio(5.0, 10.0)
        a.calculateTireSlipAngle(0.2, 15.0, 0.0)
        assert a.getTireSlipRatio() == pytest.approx(0.5)
        assert b.getTireSlipRatio() == pytest.approx(-0.5)
        assert b.getTireSlipAngle() == 0.0

    def test_instance_state_does_not_leak_into_new_tires(self):
        a = make_tire()
        solve(a, 0.3, 0.2, FZ)
        b = make_tire()
        assert b.getTireSlipRatio() == 0.0
        assert b.getTireSlipAngle() == 0.0
