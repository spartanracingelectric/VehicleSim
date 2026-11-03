import numpy as np
import matplotlib.pyplot as plt


def getSaturation(result, hold_s=0.5):
    start = None
    for i in range(len(result.time_s)):
        if result.is_saturated[i]:
            if start is None:
                start = i
            if result.time_s[i] - result.time_s[start] >= hold_s:
                return result.ay[start]
        else:
            start = None
    return None

def getRmsYawError(result, start_s):
    keep = result.time_s >= start_s
    error = result.r[keep] - result.r_ref[keep]
    return np.sqrt(np.mean(error**2))

def getShortfall(oracle, inside_cut):
    needed = np.abs(oracle.external_yaw_moment_Nm)
    available = np.interp(oracle.ay, inside_cut.ay, np.abs(inside_cut.tv_yaw_moment_Nm))
    return max(0.0, np.max(needed - available))

def getMaxSpeedHeld(result):
    return np.max(result.u)

def printAnswer(saturation_ay, shortfall_Nm, rms_error, tolerance=0.05):
    if saturation_ay is None and rms_error < tolerance:
        print("Rear inside-cut TV: YES")
    elif saturation_ay is None:
        print(f"Rear inside-cut TV: NO, yaw error {rms_error:.3f} rad/s over tolerance")
    else:
        print(f"Rear inside-cut TV: NO above {saturation_ay / 9.81:.2f} g, short {shortfall_Nm:.0f} Nm")


def show(fig, save_path):
    fig.tight_layout()
    if save_path is None:
        plt.show()
    else:
        fig.savefig(save_path, dpi=110)
    plt.close(fig)


def plotYawRate(runs, save_path=None):
    fig, ax = plt.subplots()
    for mode, result in runs.items():
        ax.plot(result.time_s, result.r, label=f"r ({mode})")
    ax.plot(result.time_s, result.r_ref, "k--", label="r_ref")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw rate (rad/s)")
    ax.grid(True)
    ax.legend()
    show(fig, save_path)


def plotYawMoment(oracle, inside_cut, saturation_ay=None, save_path=None):
    fig, ax = plt.subplots()
    ax.plot(oracle.ay, oracle.external_yaw_moment_Nm, label="needed (oracle)")
    ax.plot(inside_cut.ay, inside_cut.tv_yaw_moment_Nm, label="available (inside cut)")
    if saturation_ay is not None:
        ax.axvline(saturation_ay, color="r", ls="--", label="saturates")
    ax.set_xlabel("Lateral acceleration (m/s²)")
    ax.set_ylabel("Yaw moment (Nm)")
    ax.grid(True)
    ax.legend()
    show(fig, save_path)


def plotInsideRear(result, inside_index=2, save_path=None):
    fig, ax = plt.subplots()
    ax.plot(result.time_s, result.torque[:, inside_index], label="inside rear torque")
    ax.plot(result.time_s, result.inside_cut_Nm, label="cut")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.grid(True)
    ax.legend()
    show(fig, save_path)


def plotUndersteer(runs, save_path=None):
    fig, ax = plt.subplots()
    for mode, result in runs.items():
        ax.plot(result.ay, result.delta, label=mode)
    ax.set_xlabel("Lateral acceleration (m/s²)")
    ax.set_ylabel("Steer angle (rad)")
    ax.grid(True)
    ax.legend()
    show(fig, save_path)
