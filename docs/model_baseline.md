# VehSim-1 consolidated model baseline

This branch combines the model work available on November 3, 2026. Exact input
commits are recorded in [model_sources.json](model_sources.json). The unfinished
merge in the original TV checkout was left intact; this work uses a separate
VehSim-1 worktree.

| Source | Included |
|---|---|
| Local TV / FourMotors | Four independent electrical motor/inverter instances and actual summed battery current |
| Remote TV | Planar double-track dynamics, four wheel-speed states, load transfer, aero, steering geometry, R20 tire, torque envelope, robot driver, skidpad runner and tests |
| TireSim | Dugoff and Magic Formula tires, force filtering, longitudinal wheel slip, hydraulic braking and steering studies |
| motor/thermals | AMK winding thermal state, copper heating, coolant heat rejection, thermal validation script |
| VESC-INV-01 | VESC-style FOC current loop, current/battery limits, speed fade, field weakening, current-command stand-in VCU |
| BMSSim-1 | Cell measurement/flag helper, adapted to the existing BMSConfig thresholds |
| Tejas-01 | LocalSim Python sources and CSV resources for SR17/SR18/RIT car studies, newer tire fits, AMK efficiency map, acceleration/skidpad/autocross studies |

LocalSim is a separate study framework, not automatically part of the modular
SR18 powertrain. It is retained without replacing the modular models or importing
old copies of Vehicle/ from that branch. Generated plots, macOS metadata and
digitizer source images are not part of this consolidation. Runtime CSV model
resources are tracked even though the root ignore rule excludes CSV outputs.

## Integration boundary

```python
from copy import deepcopy
from Vehicle.MainConfigs.SR18 import SR18

car = deepcopy(SR18)
# Commands are MOTOR current in A peak. Wheel order: FL, FR, RL, RR.
car.advance(0.01, current_request_A=[10.0, 20.0, 30.0, 40.0])
torque_Nm = car.getTorqueFeedback_Nm()      # four motor-shaft torques
dc_A = car.getInverterDcCurrents_A()        # four averaged inverter DC currents
assert abs(car.battery.current_A - sum(dc_A)) < 1e-6
```

A scalar current broadcasts to all four motors. Negative commands are retained
for braking/regen. Convert VCU mA to A once at the SIL boundary. Do not convert
peak motor current to RMS before passing it to the inverter model. Motor speed
is mechanical rad/s; convert eRPM using pole pairs in the transport adapter.

`torque_request_Nm` remains a convenience input: it is converted to each motor's
iq target at id=0, then goes through the same inverter current controller. It is
not a guarantee of shaft torque once voltage limits or field weakening apply.
Setting current and torque requests simultaneously is rejected. Commands must
be finite and have exactly four entries. Motor/inverter instances must be
independent and use one common control period.

The VESC model runs at its configured current-control rate (default 26 kHz).
A 10 ms VCU interval therefore contains 260 inverter periods, not a hardcoded
100. Motor electrical substeps are adjusted to fill each inverter period exactly.

The stand-in VCU uses `getCurrentRequest_A()`. It does not duplicate the real
firmware's LC, TV, PL or safety arbitration. An external current command bypasses
the stand-in. `Tests/scenario.py` retains per-corner current, torque, DC-current,
and thermal logs as well as aggregate plotting channels.

## Two physical paths remain distinct

1. `Vehicle.MainConfigs.SR18`: four independent electrical drives and one shared
   battery, connected to the existing longitudinal axle chassis. This chassis
   still averages motor torques and uses a common motor speed. It cannot prove
   the yaw response of differential electrical commands.
2. `Vehicle.MainConfigs.SR18_skidpad`: planar dynamics with four independent wheel
   torques and wheel speeds, combined-slip tire forces, and load transfer. The
   existing runner uses the Python rear-TV research controller and a torque/
   power envelope. It does not yet include the detailed electrical drives/BMS.

The packaged runner is `Vehicle.Simulation.skidpad.runSkidpad`; the old
`Tests.skidpad` import remains compatible. A real VCU integration still needs an
adapter and the coupling between the electrical drives and planar wheel states.
Bringing all source models onto this branch does not establish that full SIL.

The active longitudinal tire is `Vehicle.Tire.TireConfig` (Dugoff). The older
`TireConfig2` is retained for compatibility. The skidpad selects its R20 Pacejka
model explicitly. The generic steering class and planar steering class are
different experimental interfaces; use the one specified by each car config.

## Parameter limits and provenance

Source configuration choices are preserved, not silently averaged. The planar
car uses the newer TV dimensions (277.6 kg including driver, 1.530 m wheelbase,
1.219/1.194 m tracks), while the longitudinal car still has its own placeholder
geometry and mass accounting. Share an approved parameter set before comparing
absolute results between these paths.

Unvalidated values include motor heat capacity, fixed coolant temperature,
inverter efficiency/VESC Tool settings, tire track-grip scale, and some geometry,
mass and control gains. The thermal model accounts for copper heat and a fixed
coolant sink; it has no core loss, coupled coolant system or thermal shutdown.
The cell BMS helper produces measurement flags; the existing BMSConfig remains
the sampled, latched contactor protection model used by SR18.

## Validation commands

```powershell
python -m pip install -e ".[dev]"
python -m pytest -q
python -m pytest Tests/scenarioTest.py -q
python Tests/motorThermalTest.py
python Tests/vescInverterTest.py
```

The default suite covers tire/brake/steering physics, skidpad left/right
symmetry, and independent current commands. Three Dugoff coupling-factor cases
skip the undefined/infinite coupling factor at exactly zero slip;
zero-slip force behavior is covered separately by the edge-case tests.
Scenario tests and the two script-based electrical/thermal checks must be run
explicitly; their filenames do not match pytest's default `test_*.py` pattern.
Tests do not establish target VCU timing, sensor hardware accuracy or on-car
validation.

The lift-off scenario retains the 0.1 Nm residual-torque criterion and allows
50 ms of inverter settling, replacing the older 20 ms torque-model assumption.
A recorded VESC replay had zero current request immediately at lift-off,
0.132 Nm peak residual after 20 ms, 0.0983 Nm after 40 ms and 0.0761 Nm after
50 ms. Wheel spin-down releases stored energy into the chassis for roughly
another 0.2 s, so monotonic body coasting is checked separately after 300 ms.
The test independently requires zero requested current at lift-off and less
than 0.1 Nm residual motor torque after 50 ms. These are model response
observations, not VCU safety-response deadlines.
