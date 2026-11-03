from copy import deepcopy
import math

import pytest

from Vehicle.MainConfigs.SR18 import SR18
from Vehicle.BMS.BMSModel import BMSModel


def test_each_motor_tracks_its_own_peak_current_and_pack_sums_dc_currents():
    car = deepcopy(SR18)
    car.battery.save_history = False
    request = [10, 20, 30, 40]
    car.advance(.02, current_request_A=request)
    assert [m.iq for m in car.motors] == pytest.approx(request, abs=.5)
    assert car.last_current_request_A == request
    assert car.battery.current_A == pytest.approx(sum(car.getInverterDcCurrents_A()))
    assert car.motors[3].getWindingTemp_C() > car.motors[0].getWindingTemp_C()
    assert all(t > 0 for t in car.getTorqueFeedback_Nm())


def test_scalar_current_is_equivalent_to_four_equal_commands():
    scalar, corners = deepcopy(SR18), deepcopy(SR18)
    scalar.advance(.005, current_request_A=20)
    corners.advance(.005, current_request_A=[20]*4)
    assert scalar.getTorqueFeedback_Nm() == pytest.approx(corners.getTorqueFeedback_Nm())
    assert scalar.battery.current_A == pytest.approx(corners.battery.current_A)


def test_signed_corner_currents_are_preserved():
    car = deepcopy(SR18)
    car.chassis.speed_mps = 10
    car.advance(.02, current_request_A=[-10, 10, -20, 20])
    assert [m.iq for m in car.motors] == pytest.approx([-10, 10, -20, 20], abs=1)
    assert car.getTorqueFeedback_Nm()[0] < 0 < car.getTorqueFeedback_Nm()[1]


@pytest.mark.parametrize("bad", [[1, 2, 3], [1, 2, 3, 4, 5], [0, 0, math.nan, 0], math.inf])
def test_bad_corner_commands_are_rejected(bad):
    with pytest.raises(ValueError, match="finite per-motor"):
        deepcopy(SR18).update(current_request_A=bad)


def test_ambiguous_current_and_torque_requests_are_rejected():
    with pytest.raises(ValueError, match="not both"):
        deepcopy(SR18).update(current_request_A=10, torque_request_Nm=2)


def test_cell_bms_uses_existing_car_threshold_configuration():
    bms = BMSModel(SR18.bms)
    healthy = bms.update([3.6]*140, [25]*140, 504, 10, 80)
    assert healthy["fault_flags"] == 0
    fault = bms.update([2.7, 4.3], [61], 504, 10, 80)
    assert fault["fault_flags"] == 0x34
