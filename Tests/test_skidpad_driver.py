from Vehicle.Driver.SkidPadDriverConfig import SkidPadDriverConfig


def makeDriver(target_speed=10.0):
    return SkidPadDriverConfig(mode="circle", target_speed_mps=lambda t: target_speed,
                               center_m=(0.0, 9.125), radius_m=9.125, turn="left",
                               lookahead_m=3.0, wheelbase_m=1.53924,
                               speed_kp=30.0, speed_ki=10.0, max_wheeltorque_Nm=248.0)


def test_on_circle_steer_is_L_over_R():
    driver = makeDriver()
    x = [0, 0, 0, 10, 0, 0, 0, 0, 0, 0]          
    delta, torque = driver.update(x, 0.0, 0.01)
    assert abs(delta - 0.1671) < 1e-4


def test_torque_rises_when_too_slow():
    driver = makeDriver()
    x = [0, 0, 0, 9, 0, 0, 0, 0, 0, 0]           
    torques = [driver.update(x, i * 0.01, 0.01)[1] for i in range(10)]
    for before, after in zip(torques, torques[1:]):
        assert after > before


def test_torque_falls_but_never_negative_when_too_fast():
    driver = makeDriver()
    x = [0, 0, 0, 11, 0, 0, 0, 0, 0, 0]          # 11 m/s, target 10
    torques = [driver.update(x, i * 0.01, 0.01)[1] for i in range(10)]
    for t in torques:
        assert t >= 0.0


def test_torque_never_above_max():
    driver = makeDriver()
    x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]           # stopped, 10 m/s too slow
    for i in range(500):
        delta, torque = driver.update(x, i * 0.01, 0.01)
        assert torque <= 248.0
