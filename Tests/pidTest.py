import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from Functions.PIDcontroller import PIDController

# run: python Tests/pidTest.py (or python -m Tests.pidTest from the repo root)
# the pid on its own, driving a simple first-order plant: dy/dt = (u - y) / TAU_S

TAU_S = 0.5
DT_S = 0.001
OUTPUT_LIMIT = 10.0

def main():
    testReachesSetpoint()
    testAntiWindup()
    testDerivative()
    print("All pid checks passed")

# clip_outside: limit the plant input outside the controller, so the controller doesn't know (no anti-windup)
def simulatePlant(controller, setpoints, clip_outside=False):
    y = 0.0
    ys = np.zeros(len(setpoints))
    integrals = np.zeros(len(setpoints))
    for step, setpoint in enumerate(setpoints):
        u = controller.update(setpoint - y, DT_S)
        if clip_outside:
            u = min(u, OUTPUT_LIMIT)
        y += (u - y) / TAU_S * DT_S
        ys[step] = y
        integrals[step] = controller.getIntegral()
    return ys, integrals

# the integral removes the steady-state error that P alone leaves
def testReachesSetpoint():
    setpoints = np.full(5000, 10.0)
    y_p, _ = simulatePlant(PIDController(Kp=2.0), setpoints)
    y_pi, _ = simulatePlant(PIDController(Kp=2.0, Ki=4.0), setpoints)

    # P only settles at Kp / (1 + Kp) of the setpoint
    assert np.isclose(y_p[-1], 10.0 * 2 / 3, rtol=1e-3)
    assert np.isclose(y_pi[-1], 10.0, rtol=1e-3)
    print(f"setpoint 10: P only settles at {y_p[-1]:.3f}, PI settles at {y_pi[-1]:.3f}")

# asks for 20 (can't reach it with the output capped at 10), then drops to 5
def testAntiWindup():
    setpoints = np.concatenate([np.full(3000, 20.0), np.full(3000, 5.0)])
    y, integrals = simulatePlant(PIDController(Kp=2.0, Ki=4.0, output_max=OUTPUT_LIMIT), setpoints)
    y_windup, integrals_windup = simulatePlant(PIDController(Kp=2.0, Ki=4.0), setpoints, clip_outside=True)

    # with anti-windup the integral holds still while the output is pinned at the limit,
    # so it gets to the new setpoint quickly. without it, the integral grows the whole time and has to unwind first
    assert np.ptp(integrals[:3000]) == 0
    assert abs(y[-1] - 5.0) < 0.05
    assert y_windup[-1] > 9.0
    print(f"after the drop to 5: anti-windup y = {y[-1]:.2f} (integral {integrals[2999]:.1f}), "
          f"no anti-windup y = {y_windup[-1]:.2f} (integral wound up to {integrals_windup[2999]:.1f})")

def testDerivative():
    controller = PIDController(Kp=0.0, Kd=0.5)
    controller.update(0.0, 0.01)
    output = controller.update(1.0, 0.01)
    assert np.isclose(output, 0.5 * 1.0 / 0.01)
    print(f"derivative: error jumps by 1 in 0.01 s -> Kd term {output:.1f}")

if __name__ == "__main__":
    main()
