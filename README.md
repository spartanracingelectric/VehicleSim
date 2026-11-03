# VehicleSim

`VehSim-1` is the consolidated SR18 model baseline. It includes four independent
AMK/VESC electrical drives, the shared battery/BMS, tire and brake models,
motor winding thermals, and the planar skidpad model. Source commits, parameter
assumptions and remaining integration gaps are documented in
[the model baseline](docs/model_baseline.md).

Install Python 3.10+ and then run from this repository:

```powershell
python -m pip install -e ".[dev]"
python -m pytest -q
python -m pytest Tests/scenarioTest.py -q
python Tests/motorThermalTest.py
python Tests/vescInverterTest.py
```

For an external VCU, supply signed peak motor-current commands in FL/FR/RL/RR order:

```python
from copy import deepcopy
from Vehicle.MainConfigs.SR18 import SR18

car = deepcopy(SR18)
car.advance(0.01, current_request_A=[10, 20, 30, 40])
print(car.getTorqueFeedback_Nm())       # four motor-shaft torques, Nm
print(car.getInverterDcCurrents_A())    # four actual averaged DC currents, A
```

A scalar current broadcasts to every motor. The car sums the four DC currents
into its battery. Invalid command lengths/nonfinite values are rejected. Passing
external commands bypasses the simple Python pedal controller; actual VCU control
logic belongs in SRE-VCU.

The detailed electrical car currently uses the longitudinal chassis; differential
commands do not yet drive four independent planar wheel states. The separate
planar model accepts four wheel torques and supports steering, slip, yaw, aero
and load transfer. Its packaged runner is
`Vehicle.Simulation.skidpad.runSkidpad`; study examples are in `Tests/skidpadTest.py`.
The electrical and planar paths still need to be coupled for full VCU TV SIL.

`LocalSim/` contains the latest Tejas-01 car/tire/efficiency-map study sources and
runtime CSV resources. It is a separate research framework, not the default
modular car or an installed package. Generated plots and digitizer source images
remain on the source branch.

See [model_sources.json](docs/model_sources.json) for the exact consolidated
revisions. These are current source models, with documented placeholders; they
have not all been validated against the physical car.
