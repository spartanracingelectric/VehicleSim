"""The Burckhardt-fade R20 tire: same as tire_r20 until the peak, then grip fades as the wheel spins."""
# ai made this
import pytest

from Vehicle.Tire.PacejkaR20TireConfig import PacejkaR20TireConfig
from Vehicle.Tire.BurckhardtR20TireConfig import BurckhardtR20TireConfig


FZ = 700.0 # N, about one tire's load on the SR18
def make_pair():
    plain = PacejkaR20TireConfig(psi=12.0, track_grip_scale=0.6)
    faded = BurckhardtR20TireConfig(psi=12.0, track_grip_scale=0.6, surface="dry_asphalt")
    return plain, faded

def test_same_as_plain_r20_before_the_peak():
    plain, faded = make_pair()
    for slip in (0.0, 0.02, 0.05, 0.08, 0.1):
        assert faded.calculateForces(FZ, slip, 0.0)[0] == pytest.approx(plain.calculateForces(FZ, slip, 0.0)[0])


def test_keeps_65_percent_of_peak_at_full_wheelspin():
    _, faded = make_pair()
    fx, _ = faded.calculateForces(FZ, 1.0, 0.0)
    assert fx / faded.getPeakLongitudinal_N(FZ) == pytest.approx(0.65, abs=0.005)


def test_stops_fading_past_slip_1():
    _, faded = make_pair()
    assert faded.calculateForces(FZ, 3.0, 0.0)[0] == pytest.approx(faded.calculateForces(FZ, 1.0, 0.0)[0])


def test_more_spin_never_gives_more_push_after_the_peak():
    _, faded = make_pair()
    forces = [faded.calculateForces(FZ, s / 100, 0.0)[0] for s in range(15, 101)]
    assert all(later <= earlier + 1e-9 for earlier, later in zip(forces, forces[1:]))


def test_locked_wheel_is_the_mirror_of_spinning_wheel():
    _, faded = make_pair()
    assert faded.calculateForces(FZ, -1.0, 0.0)[0] == pytest.approx(-faded.calculateForces(FZ, 1.0, 0.0)[0])


def test_sideways_grip_is_unchanged():
    plain, faded = make_pair()
    for angle in (0.05, 0.2, 0.4):
        assert faded.calculateForces(FZ, 0.0, angle)[1] == pytest.approx(plain.calculateForces(FZ, 0.0, angle)[1])


def test_grip_limit_flag_while_spinning():
    _, faded = make_pair()
    faded.calculateForces(FZ, 0.02, 0.0)
    assert not faded.isAtGripLimit()
    faded.calculateForces(FZ, 0.6, 0.0)
    assert faded.isAtGripLimit()


def test_works_with_the_longitudinal_chassis_call():
    _, faded = make_pair()
    fx, fy = faded.calculateForces(FZ, 0.3, 0.0, dt_s=1e-4)  # ChassisConfig passes dt_s
    assert fx > 0.0

