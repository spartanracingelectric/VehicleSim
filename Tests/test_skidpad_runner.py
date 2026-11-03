import numpy as np
from copy import deepcopy

from Vehicle.MainConfigs.SR18_skidpad import SR18_skidpad
from Vehicle.VCU.Configs.reartv1 import reartv1
from Tests.skidpad import runSkidpad
from Tests.skidpadTest import makeDriver
from types import SimpleNamespace
from Data import skidpadAnalysis as analysis


RUN_TIME_S = 10.0   


def runOne(turn):
    tv = deepcopy(reartv1)
    tv.mode = "inside_cut"
    return runSkidpad(SR18_skidpad, makeDriver("B", turn), tv, RUN_TIME_S, start_speed_mps=6.0)


def test_left_and_right_mirror():
    left = runOne("left")
    right = runOne("right")

    assert left["stop_reason"] == right["stop_reason"]
    assert len(left["time_s"]) == len(right["time_s"])

    for name in ["u_mps", "radius_error_m", "X_m"]:
        assert np.allclose(left[name], right[name], rtol=0.01, atol=1e-3), name


    for name in ["Y_m", "psi_rad", "r_radps", "delta_rad", "ay_mps2"]:
        assert np.allclose(left[name], -right[name], rtol=0.01, atol=1e-3), name

