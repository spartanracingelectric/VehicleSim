# Skidpad runs A and B, both directions, each TV mode. Run with: python Tests/skidpadTest.py
import math
import sys
from copy import deepcopy
import numpy as np
from types import SimpleNamespace


from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1])) 

from Vehicle.MainConfigs.SR18_skidpad import SR18_skidpad
from Vehicle.VCU.Configs.reartv1 import reartv1
from Vehicle.Driver.SkidPadDriverConfig import SkidPadDriverConfig
from Tests.skidpad import runSkidpad
from Data import skidpadAnalysis as analysis

CAR = SR18_skidpad
RADIUS_M = 9.125
RUN_TIME_S = 10.0

# run name depnding on which TV mode u wanna try
RUNS = {
    "A": ["off", "inside_cut", "oracle"],
    "B": ["off", "inside_cut"],
}


def makeDriver(run, turn):
    if turn == "left":
        sign = 1
    
    else: 
        sign = -1

    center = (0.0, sign * RADIUS_M)
    common = dict(radius_m=RADIUS_M, turn=turn, lookahead_m=3.0, wheelbase_m=CAR.wheelbase,
                  speed_kp=30.0, speed_ki=10.0, max_wheeltorque_Nm=248.0, center_m=center)
    if run == "A":
        # A: steer held at wheelbase/turning radius, speed is fucntion ramping up from 5 m/s to .2 m/s^2 until something stops it
        steer = sign * CAR.wheelbase / RADIUS_M
        return SkidPadDriverConfig(mode="fixed", target_speed_mps=lambda t: 5.0 + 0.2 * t,
                                   delta_rad=lambda t: steer, **common)
        # B: pure pursuit in circular track speed steps from 6m/s, with a +.5 m/s recovery every 4 seconds
    return SkidPadDriverConfig(mode="circle", target_speed_mps=lambda t: 6.0 + 0.5 * (t // 4), **common)


def runAll():
    results = {}
    for run, modes in RUNS.items():
        start_speed = 5.0 if run == "A" else 6.0
        for mode in modes:
            for turn in ("left", "right"):
                tv = deepcopy(reartv1)
                tv.mode = mode
                driver = makeDriver(run, turn)
                results[(run, mode, turn)] = runSkidpad(CAR, driver, tv, RUN_TIME_S, start_speed_mps=start_speed)
    return results


def forAnalysis(r, skip_s=1.0):
    # keep only ticks after the startup spike (first second)
    keep = []
    for i in range(len(r["time_s"])):
        if r["time_s"][i] >= skip_s:
            keep.append(i)

    # build the object skidpadAnalysis expects: his short names, our logged columns
    result = SimpleNamespace()
    result.time_s = r["time_s"][keep]
    result.ay = r["ay_mps2"][keep]
    result.r = r["r_radps"][keep]
    result.r_ref = r["r_ref_radps"][keep]
    result.u = r["u_mps"][keep]
    result.delta = r["delta_rad"][keep]
    result.torque = r["wheel_torques_Nm"][keep]
    result.is_saturated = r["is_saturated"][keep]
    result.inside_cut_Nm = r["inside_cut_Nm"][keep]
    result.external_yaw_moment_Nm = r["external_yaw_moment_Nm"][keep]
    result.tv_yaw_moment_Nm = r["tv_yaw_moment_Nm"][keep]
    return result


def main():
    results = runAll()
    out = Path(__file__).resolve().parents[1] / "Data" / "Outputs"
    out.mkdir(parents=True, exist_ok=True)

    # left-turn runs only (the mirror test showed right is the same, flipped), first second dropped
    A = {}
    for mode in RUNS["A"]:
        A[mode] = forAnalysis(results[("A", mode, "left")])
    B = {}
    for mode in RUNS["B"]:
        B[mode] = forAnalysis(results[("B", mode, "left")])

    saturation_ay = analysis.getSaturation(A["inside_cut"])
    shortfall = analysis.getShortfall(A["oracle"], A["inside_cut"])
    rms_off = analysis.getRmsYawError(A["off"], start_s=1.0)
    rms_tv = analysis.getRmsYawError(A["inside_cut"], start_s=1.0)

    print()
    print(f"RMS yaw error, run A:  off {rms_off:.3f}   inside_cut {rms_tv:.3f} rad/s")
    print(f"Top speed, run B:      off {analysis.getMaxSpeedHeld(B['off']):.2f}   "
          f"inside_cut {analysis.getMaxSpeedHeld(B['inside_cut']):.2f} m/s")
    analysis.printAnswer(saturation_ay, shortfall, rms_tv)

    analysis.plotYawRate(A, save_path=out / "yaw_rate_runA.png")
    analysis.plotYawMoment(A["oracle"], A["inside_cut"], saturation_ay, save_path=out / "yaw_moment.png")
    analysis.plotInsideRear(A["inside_cut"], save_path=out / "inside_rear_runA.png")
    analysis.plotUndersteer(B, save_path=out / "understeer_runB.png")
    print(f"plots saved to {out}")


    print(f"{'run':4}{'TV':12}{'turn':7}{'stopped':16}{'at':>8}{'top speed':>12}{'max ay':>10}")
    for (run, mode, turn), r in results.items():
        max_ay_g = max(abs(a) for a in r["ay_mps2"]) / 9.81
        print(f"{run:4}{mode:12}{turn:7}{r['stop_reason']:16}{r['time_s'][-1]:7.2f}s"
              f"{max(r['u_mps']):10.2f} m/s{max_ay_g:8.2f} g")


if __name__ == "__main__":
    main()
