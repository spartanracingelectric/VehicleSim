"""run ts for acceleration with grip penalty gang

python -m examples.burckhardt_launch
"""
from copy import deepcopy

import numpy as np
import matplotlib.pyplot as plt

from Vehicle import create_sr18
from Vehicle.Simulation.pedals import ramp
from Vehicle.Simulation.scenario import Scenario, runScenario
from Vehicle.Tire.Configs.tire1 import tire1
from Vehicle.Tire.Configs.tire_r20_burckhardt import tire_r20_burckhardt


def launch(tire):
    car = create_sr18()
    car.tire = deepcopy(tire)
    run = Scenario("75 m launch", pedal=ramp(to=100, over_s=0.8), run_time_s=6.0,
                   stop_at_distance_m=75, vehicle=car, record_every_s=0.001)
    return runScenario(run)


def main():
    fig, (force_plot, speed_plot) = plt.subplots(1, 2, figsize=(11, 4))

    # 1. the tire by itself: push force vs slip at one tire's load
    slips = np.linspace(-1.2, 1.2, 600)
    for tire in (tire1, tire_r20_burckhardt):
        tire = deepcopy(tire)
        force_plot.plot(slips, [tire.calculateForces(700.0, s, 0.0)[0] for s in slips], label=tire.model_name)
    force_plot.set(xlabel="slip ratio", ylabel="Fx at 700 N load (N)", title="Push force vs slip")
    force_plot.grid(True)
    force_plot.legend()

    # 2. the whole car: full-throttle launch to 75 m
    for tire in (tire1, tire_r20_burckhardt):
        result = launch(tire)
        print(f"{tire.model_name:32s} 75 m in {result.time_s[-1]:.3f} s")
        speed_plot.plot(result.time_s, result.speed_mps * 3.6, label=tire.model_name)
    speed_plot.set(xlabel="time (s)", ylabel="speed (km/h)", title="Full-throttle launch")
    speed_plot.grid(True)
    speed_plot.legend()

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
