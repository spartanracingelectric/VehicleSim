# VehicleSim

SR18 physical models for controller development. Start with a fresh model:

```python
from Vehicle import create_sr18, create_skidpad

car = create_sr18()
car.advance(0.01, current_request_A=[10, 20, 30, 40])  # A peak, FL/FR/RL/RR
print(car.getTorqueFeedback_Nm())                    # motor-shaft Nm
```

Choose the model by the behavior you need:

| Constructor | Simulates | Current limitation |
|---|---|---|
| `create_sr18()` | Four independent AMK/VESC electrical drives, battery/BMS, thermal states, longitudinal chassis and brakes | Chassis averages drive torques; no differential-drive yaw |
| `create_skidpad()` | Planar motion, four wheel speeds/torques, tire forces, load transfer, aero and steering | Electrical drives and battery are not coupled to it yet |

These are explicit fidelity choices. Full VCU TV SIL still requires their
coupling and the actual SRE-VCU adapter. Do not use the electrical model alone
to claim torque-vectoring yaw performance.

## Install and verify

```powershell
python -m pip install -e ".[dev]"
python -m pytest -q
```

One command discovers unit physics checks, motor/inverter and thermal checks,
and all six straight-line scenarios. Three undefined zero-slip Dugoff factor
cases intentionally skip; zero-slip force behavior is tested separately.

## Repository layout

| Location | Purpose |
|---|---|
| `Vehicle/` | Supported physical models, explicit car factories and runtime simulation helpers |
| `Functions/` | Numerical helpers used by those models |
| `Data/` | Log readers and plotting/analysis utilities |
| `tests/` | Discovered regression tests; no runtime code depends on this directory |
| `examples/` | Runnable plotting and simulation demonstrations |
| `research/tejas/` | Independent Tejas car/tire studies and their input data |
| `research/legacy/` | Retired implementations and incomplete placeholders |
| `docs/` | Model assumptions, interface, migration notes and source provenance |
| `outputs/` | Generated figures/logs, ignored by Git |

The installed package excludes tests, examples, research and historical notes.
Each factory owns its mutable state and explicitly selects its configurations;
creating a car does not scan/import every configuration in the repository.

Run demonstrations as modules, for example:

```powershell
python -m examples.battery --duration-s 1
python -m examples.scenarios --help
python -m examples.motor_inverter
python -m examples.skidpad
```

See [model assumptions and interface](docs/model_baseline.md),
[layout migration](docs/repository_layout.md), and
[exact model source commits](docs/model_sources.json).
