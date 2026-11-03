from Vehicle.VCU.RearTVConfig import RearTVConfig

U = 11.588           # 1.5 g on the skidpad
DELTA = 0.16868      # L / R


def makeTV():
    # a fresh controller: on its first update the filter starts at r_ref, so it's already settled
    return RearTVConfig(kp_Nm_per_radps=200.0, ki_Nm_per_rad=0.0, wheelbase_m=1.53924,
                        ref_filter_time_s=0.05, understeer_gradient=0.0, mode="inside_cut")


def state(u, r):
    return [0, 0, 0, u, 0, r, 0, 0, 0, 0]


def test_r_ref_on_skidpad():
    tv = makeTV()
    tv.update(state(U, 1.27), DELTA, 100.0, 0.01)
    assert abs(tv.r_ref_radps - 1.2699) < 1e-3


def test_left_turn_understeer_cuts_inside_rear():
    tv = makeTV()
    torques, moment = tv.update(state(U, 1.10), DELTA, 100.0, 0.01)
    assert abs(tv.inside_cut_Nm - 34.0) < 0.1
    assert abs(torques[2] - 66.0) < 0.1              # RL is the inside rear
    assert torques[0] == 100.0 and torques[1] == 100.0 and torques[3] == 100.0


def test_left_turn_oversteer_cuts_nothing():
    tv = makeTV()
    torques, moment = tv.update(state(U, 1.40), DELTA, 100.0, 0.01)
    assert tv.inside_cut_Nm == 0.0
    assert torques == [100.0, 100.0, 100.0, 100.0]


def test_right_turn_cuts_RR():
    tv = makeTV()
    torques, moment = tv.update(state(U, -1.10), -DELTA, 100.0, 0.01)
    assert abs(torques[3] - 66.0) < 0.1              # RR is the inside rear now
    assert torques[0] == 100.0 and torques[1] == 100.0 and torques[2] == 100.0


def test_cut_clamped_at_base_and_saturated():
    tv = makeTV()
    torques, moment = tv.update(state(U, 1.10), DELTA, 20.0, 0.01)
    assert abs(torques[2] - 0.0) < 1e-9              # inside rear cut all the way to 0
    assert torques[0] == 20.0 and torques[1] == 20.0 and torques[3] == 20.0
    assert tv.is_saturated
