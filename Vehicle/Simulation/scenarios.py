import argparse
from pathlib import Path
from Vehicle.Simulation.scenario import Scenario, runScenario, summarize, plotScenario, stopWhenBmsShutsDown
from Vehicle.Simulation.pedals import hold, ramp, steps, pulses
from Vehicle.Simulation.validation import (ALWAYS, torqueAbout, torqueNeverAbove, accelerates, coasts, staysParked,
                          reachesDistance, reachesSpeed, noOverpower, overpowerFlagged, noShutdown, shutsDown,
                          zeroCurrentRequest, residualTorqueBelow)

# one Scenario = one test: the pedal over time, how the car starts, when it stops, and what has to be true.
# checks.ALWAYS (inverter current limit, vcu timing, bms flag) runs on every scenario on top of its own checks.
# to add a test, add a row: pedal shapes are in Vehicle/Simulation/pedals.py, checks in Vehicle/Simulation/validation.py

SCENARIOS = [
    Scenario("part throttle 30%",
             pedal=hold(30), run_time_s=2.0,
             checks=[torqueAbout(0.30 * 21.0, after_s=0.05), accelerates(), noOverpower(), noShutdown()]),

    # VESC field-weakening/current state decays after the request goes to zero.
    # The recorded replay is below 0.1 Nm by 40 ms; allow 50 ms to settle.
    # Spinning wheels can still push while releasing stored energy: test body
    # coasting after 300 ms, independently of electrical command removal.
    Scenario("lift off at 2 s",
             pedal=steps((0.0, 100), (2.0, 0)), run_time_s=3.0,
             checks=[zeroCurrentRequest(after_s=2.0), residualTorqueBelow(.1, after_s=2.05),
                     coasts(after_s=2.3), noShutdown()]),

    Scenario("accel event 75 m",
             pedal=ramp(to=100, over_s=0.8), run_time_s=10.0, stop_at_distance_m=75,
             checks=[reachesDistance(75, within_s=4.5), reachesSpeed(100), accelerates(), overpowerFlagged(), noShutdown()]),

    Scenario("low battery",
             pedal=ramp(to=100, over_s=0.8), run_time_s=2.0, start_soc=0.02,
             checks=[shutsDown(by_s=1.0)]),

    Scenario("hot pack won't drive",
             pedal=ramp(to=100, over_s=0.8), run_time_s=0.5, start_temp_C=61.5,
             checks=[shutsDown(by_s=0.01), staysParked()]),

    Scenario("cold pack won't drive",
             pedal=ramp(to=100, over_s=0.8), run_time_s=0.5, start_temp_C=-5.0,
             checks=[shutsDown(by_s=0.01), staysParked()]),
]

# long runs, only with --slow or when named (pytest skips these)
SLOW_SCENARIOS = [
    # flat out until the bms stops it (undervoltage when the pack runs down, or overtemperature, it has no cooling)
    Scenario("full throttle until the battery runs out",
             pedal=ramp(to=100, over_s=0.8), run_time_s=1800.0, stop_when=stopWhenBmsShutsDown,
             record_every_s=0.01, checks=[shutsDown()]),
]

def checkResult(scenario, result):
    for rule in ALWAYS + scenario.checks:
        rule(result)

def runAndCheck(scenario):
    result = runScenario(scenario)
    checkResult(scenario, result)
    return result

# without pytest: run them and print pass/fail, optionally plotting each one
def main():
    parser = argparse.ArgumentParser(description="run the straight-line scenarios")
    parser.add_argument("names", nargs="*", help="only run scenarios with any of these in their name")
    parser.add_argument("--plot", action="store_true", help="open a plot of each scenario")
    parser.add_argument("--save", metavar="FOLDER", help="save a plot of each scenario into this folder")
    parser.add_argument("--slow", action="store_true", help="also run SLOW_SCENARIOS (minutes each)")
    args = parser.parse_args()

    # slow ones only run with --slow, or when you ask for them by name
    candidates = SCENARIOS + (SLOW_SCENARIOS if args.slow or args.names else [])
    chosen = [scenario for scenario in candidates
              if not args.names or any(name.lower() in scenario.name.lower() for name in args.names)]
    if not chosen:
        print(f"no scenario has {args.names} in its name")
        return

    failed = 0
    for scenario in chosen:
        result = runScenario(scenario)
        try:
            checkResult(scenario, result)
            print(f"PASS  {summarize(result)}")
        except AssertionError as error:
            failed += 1
            print(f"FAIL  {scenario.name}: {error}")

        # plots even when it fails, that's usually when you want to see it
        if args.save:
            folder = Path(args.save)
            folder.mkdir(parents=True, exist_ok=True)
            file_name = "".join(c if c.isalnum() else "_" for c in scenario.name).strip("_") + ".png"
            plotScenario(result, folder / file_name)
        if args.plot:
            plotScenario(result)

    print(f"{len(chosen) - failed}/{len(chosen)} scenarios passed")

if __name__ == "__main__":
    main()
