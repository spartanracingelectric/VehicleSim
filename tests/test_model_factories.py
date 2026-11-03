from Vehicle import create_sr18, create_skidpad


def test_electrical_cars_do_not_share_simulation_state():
    first, second = create_sr18(), create_sr18()
    first.battery.soc = .2
    first.motors[0].iq = 123
    first.inverters[0].current_max_A = 42
    first.chassis.speed_mps = 10
    assert second.battery.soc != .2
    assert second.motors[0].iq == 0
    assert second.inverters[0].current_max_A != 42
    assert second.chassis.speed_mps == 0


def test_planar_cars_do_not_share_tire_and_limit_state():
    first, second = create_skidpad(), create_skidpad()
    first.tire.track_grip_scale = .1
    first.limits.power_limit_W = 1
    assert second.tire.track_grip_scale != .1
    assert second.limits.power_limit_W != 1
