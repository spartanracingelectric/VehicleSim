import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
import matplotlib.pyplot as plt
from Functions.loadConfigs import loadConfigs

# run: python Tests/tireTest.py (or python -m Tests.tireTest from the repo root)
# every tire in Vehicle/Tire/Configs (dugoff tire1, magic formula tire1_mf, ...) on the same plots:
# Fx vs slip ratio (straight line) and Fy vs slip angle (cornering), at the TTC fit load
# (nelson's version plotted TireConfig2's dugoff, tire1 is the new TireConfig dugoff now)

#TODO:
#get the right coefficients for c_long/lat/tiremu dependent on load
#Create a for loop on Fz to test tire load sensitivity
#split latmu and longmu

FZ_N = 667.0    # the load the TTC data was fit at

def main():
    tires = vars(loadConfigs("Vehicle.Tire.Configs"))
    slip_ratios = np.linspace(-1.0, 1.0, 401)
    slip_angles = np.linspace(-0.5, 0.5, 401)

    fig, (fx_ax, fy_ax) = plt.subplots(1, 2, figsize=(12, 5))
    for name, tire in sorted(tires.items()):
        label = f"{name} ({tire.model_name})"
        fx_ax.plot(slip_ratios, [tire.calculateForces(FZ_N, k, 0.0)[0] for k in slip_ratios], label=label)
        fy_ax.plot(np.degrees(slip_angles), [tire.calculateForces(FZ_N, 0.0, a)[1] for a in slip_angles], label=label)

    fx_ax.set_xlabel("Slip Ratio, kappa")
    fx_ax.set_ylabel("Longitudinal Force, Fx [N]")
    fx_ax.set_title(f"Fx vs slip ratio, Fz = {FZ_N:.0f} N")
    fy_ax.set_xlabel("Slip Angle, alpha [deg]")
    fy_ax.set_ylabel("Lateral Force, Fy [N]")
    fy_ax.set_title(f"Fy vs slip angle, Fz = {FZ_N:.0f} N")
    for ax in (fx_ax, fy_ax):
        ax.grid()
        ax.legend()
    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
