from Vehicle.Motor.Configs.amk_limits import amk_limits


def test_max_torque_flat_part_of_curve():
    # 11.6 m/s: wheel 57.09 rad/s -> motor 6438 rpm -> 21 Nm x 11.81
    assert abs(amk_limits.getMaxWheelTorque_Nm(57.09) - 248.0) < 0.2


def test_curve_limits_torque():
    # asks for 300, curve only allows 248; power is under 80 kW
    capped = amk_limits.capWheelTorques_Nm([300] * 4, [57.09] * 4)
    for t in capped:
        assert abs(t - 248.0) < 0.2


def test_power_limit_scales_torque():
    # 20 m/s: 4 x 248 Nm = 97.6 kW > 80 kW, scaled down to 203.2 each
    capped = amk_limits.capWheelTorques_Nm([248] * 4, [98.43] * 4)
    for t in capped:
        assert abs(t - 203.2) < 0.2


def test_max_torque_falling_part_of_curve():
    # 30 m/s: motor 16650 rpm -> 9.14 Nm x 11.81
    assert abs(amk_limits.getMaxWheelTorque_Nm(147.64) - 107.9) < 0.2


def test_no_regen():
    capped = amk_limits.capWheelTorques_Nm([100, 100, 100, -20], [57.09] * 4)
    expected = [100, 100, 100, 0]
    for t, e in zip(capped, expected):
        assert abs(t - e) < 0.2
