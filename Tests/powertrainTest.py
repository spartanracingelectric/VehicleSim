import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))    # repo root, so this also runs as a file or with the run button
import numpy as np
from Vehicle.MainConfigs.SR17 import SR17
from Data.plot import plotAcceleration
from Tests.scenario import Scenario, runScenario
from Tests.pedals import ramp
from Tests.checks import ALWAYS, accelerates, overpowerFlagged

# run: python Tests/powertrainTest.py (or python -m Tests.powertrainTest from the repo root)
# acceleration run: the driver gets on the throttle (0 -> 100% over THROTTLE_RAMP_TIME_S) and holds it.
# pedal -> vcu (10 ms loop) -> inverters -> motors -> battery -> bms, and the motor torque moves the car.
# power isn't limited (the flashed VCU does that), the bms flags when the pack goes over 80 kW

VEHICLE = SR17                # swap in another car config here
RUN_TIME_S = 15.0             # how long the driver holds full throttle
THROTTLE_RAMP_TIME_S = 0.8    # 0 -> 100% pedal, the fastest stab in the Crows log was 10 -> 90% in 0.64 s
ACCEL_EVENT_M = 75.0          # length of the fsae acceleration event
LOW_SOC = 0.02                # nearly empty pack, sags under load until the bms shuts it down
LOW_SOC_RUN_TIME_S = 2.0

def main():
    result = accelerate(VEHICLE, RUN_TIME_S)
    checkAcceleration(result)
    checkShutdown(accelerate(VEHICLE, LOW_SOC_RUN_TIME_S, initial_soc=LOW_SOC))
    print("All powertrain checks passed")

    plotAcceleration(
        result.time_s,
        result.apps_percent,
        result.torque_request_Nm,
        result.torque_feedback_Nm,
        result.speed_mps * 3.6,
        result.motor_rpm,
        result.pack_power_W / 1000,
        result.power_threshold_kW,
        result.overpower,
        result.pack_voltage_V,
        result.pack_current_A,
        f"{result.num_motors} motors, full throttle for {RUN_TIME_S:.0f} s",
    )

# the driver: pedal goes from 0 to 100% over THROTTLE_RAMP_TIME_S, then stays down
# (the loop itself lives in Tests/scenario.py, shared with Tests/scenarioTest.py)
def accelerate(vehicle, run_time_s, initial_soc=None):
    return runScenario(Scenario(
        "acceleration",
        pedal=ramp(to=100, over_s=THROTTLE_RAMP_TIME_S),
        run_time_s=run_time_s,
        start_soc=initial_soc,
        vehicle=vehicle,
    ))

def checkAcceleration(result):
    assert not result.shutdown.any(), "full pack shouldn't fault"
    pack_power_kW = result.pack_power_W / 1000
    speed_kph = result.speed_mps * 3.6

    # same rules as the scenario tests (Tests/checks.py): vcu only changes its request on its 10 ms loop,
    # torque never over the motor max, the bms flag matches what it measured, the car keeps speeding up
    # until it tops out, and the pack goes over 80 kW at some point (nothing here limits it)
    for rule in ALWAYS + [accelerates(), overpowerFlagged()]:
        rule(result)

    # once the pedal is down the request is full torque
    full_pedal = result.time_s >= THROTTLE_RAMP_TIME_S + result.vcu_loop_period_s
    assert np.all(result.torque_request_Nm[full_pedal] == result.max_torque_request_Nm)
    top = np.argmax(result.speed_mps)

    # results
    accel_event = np.argmax(result.distance_m >= ACCEL_EVENT_M)
    print(f"{ACCEL_EVENT_M:.0f} m (accel event): {result.time_s[accel_event]:.2f} s, {speed_kph[accel_event]:.0f} km/h")
    if np.any(speed_kph >= 100):
        print(f"0-100 km/h: {result.time_s[np.argmax(speed_kph >= 100)]:.2f} s")
    print(f"top speed {speed_kph[top]:.1f} km/h at {result.time_s[top]:.2f} s ({result.motor_rpm[top]:.0f} rpm)")
    print(f"after {result.time_s[-1] + result.control_period_s:.0f} s: {speed_kph[-1]:.0f} km/h, "
          f"{result.distance_m[-1]:.0f} m, motors at {result.motor_rpm[-1]:.0f} rpm "
          f"({np.max(result.torque_feedback_Nm):.2f} Nm max, {result.torque_feedback_Nm[-1]:.2f} Nm at the end)")

    if result.traction_limited.any():
        traction_time_s = result.traction_limited.sum() * result.control_period_s
        last_traction_s = result.time_s[np.nonzero(result.traction_limited)[0][-1]]
        print(f"tires at their grip limit for {traction_time_s:.2f} s (until {last_traction_s:.2f} s, "
              f"{speed_kph[np.nonzero(result.traction_limited)[0][-1]]:.0f} km/h): "
              f"more torque than the tires can use, the rest would be wheelspin")
    if result.overpower.any():
        overpower_time_s = result.overpower.sum() * result.control_period_s
        print(f"bms overpower flag from {result.time_s[np.argmax(result.overpower)]:.2f} s, "
              f"up for {overpower_time_s:.2f} s, peak {np.max(pack_power_kW):.1f} kW "
              f"at {result.time_s[np.argmax(pack_power_kW)]:.2f} s")
    if result.voltage_saturated.any():
        first_saturated = np.argmax(result.voltage_saturated)
        print(f"inverter hits its voltage limit at {result.time_s[first_saturated]:.2f} s "
              f"({result.motor_rpm[first_saturated]:.0f} rpm, {speed_kph[first_saturated]:.0f} km/h)")
    print(f"pack {result.pack_voltage_V[0]:.0f} V -> {np.min(result.pack_voltage_V):.0f} V at "
          f"{np.max(result.pack_current_A):.0f} A, SOC {result.soc[0]:.3f} -> {result.soc[-1]:.3f}, "
          f"{result.pack_temp_C[0]:.1f} -> {result.pack_temp_C[-1]:.1f} C")

# nearly empty pack: cells sag under the undervoltage threshold, bms opens the contactors and it stays latched
def checkShutdown(result):
    assert result.shutdown.any(), "low pack should fault"
    first = np.argmax(result.shutdown)
    after = slice(first + 1, None)

    assert result.shutdown[after].all()
    assert np.all(result.torque_feedback_Nm[after] == 0)
    assert np.all(result.pack_current_A[after] == 0)

    # with no torque the car just coasts, drag and rolling resistance slow it down
    assert np.all(np.diff(result.speed_mps[after]) <= 1e-12)

    # the cells recover once the current stops, but the shutdown stays latched
    assert result.min_cell_voltage_mV[-1] > 2800
    print(f"low pack (SOC {LOW_SOC}): cell sagged to {result.min_cell_voltage_mV[first]:.0f} mV at "
          f"{result.time_s[first]:.2f} s ({result.speed_mps[first] * 3.6:.0f} km/h) -> bms shutdown, "
          f"torque and current 0 from then on and the car coasts, "
          f"cells recovered to {result.min_cell_voltage_mV[-1]:.0f} mV but it stays off")

if __name__ == "__main__":
    main()
