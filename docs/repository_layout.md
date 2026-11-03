# Repository layout and migration

The cleanup separates supported runtime models from regression tests, plotting
examples and research archives. It changes organization and model construction,
not the physical parameter choices or controller behavior.

## Start here

```python
from Vehicle import create_sr18, create_skidpad
electrical_car = create_sr18()
planar_car = create_skidpad()
```

Factories create independent battery, inverter, motor, tire and controller state.
The electrical factory explicitly imports its selected configuration modules;
it no longer executes every module in every Configs directory. Alternate tire
models remain available by explicit selection, not accidental directory order.

## Moved locations

| Previous location | Current location |
|---|---|
| `Tests/` regression files and script-only checks | `tests/test_*.py`, all discovered by pytest |
| `Tests/scenario.py`, `pedals.py`, `checks.py` | `Vehicle/Simulation/scenario.py`, `pedals.py`, `validation.py` |
| `Tests/scenarioTest.py` catalog/CLI | `Vehicle/Simulation/scenarios.py`; CLI: `python -m examples.scenarios` |
| `Tests/motorInverterTest.py` | `Vehicle/Simulation/dyno.py`; plotting: `python -m examples.motor_inverter` |
| `Tests/skidpad.py` wrapper | Import `Vehicle.Simulation.skidpad.runSkidpad` directly |
| `Tests/skidpadTest.py`, `powertrainTest.py`, `pwmTest.py`, `tireTest.py` | `examples/skidpad.py`, `powertrain.py`, `pwm.py`, `tires.py` |
| `battery_sim.py` | `examples/battery.py` |
| `LocalSim/` | `research/tejas/`, with relative study resources preserved |
| `simrunner.py` | `research/legacy/trace_energy.py` |
| `Vehicle/Tire/TireConfig2.py` | `research/legacy/dugoff_original.py` |
| `Vehicle/Motor/Configs/emrax_228.py` | `research/legacy/emrax_placeholder.py` |
| `Functions/ODEsolver.py` comment-only placeholder | `research/legacy/ODEsolver.py` |
| `Vehicle/Inverter/Configs/inverter1.py` alias | `research/legacy/inverter_alias.py`; use `vesc_srinverter` directly |
| Root writeups and previous validation summary | `docs/history/` |
| `pytest.ini` | `[tool.pytest.ini_options]` in `pyproject.toml` |

Old test and example paths intentionally stop being public imports. Update local
scripts using the table above. Existing `Vehicle.MainConfigs` model-object
imports still work, and the old chassis steering import forwards to the single
supported steering implementation.

## Keep the boundary clean

- Runtime package code must not import `tests`, `examples` or `research`.
- Add regression tests as `tests/test_*.py`; run everything with
  `python -m pytest -q`. Electrical/thermal and six vehicle scenarios now run
  through that command too.
- Run examples with `python -m examples.<name>` from the checkout. Generated
  plots/logs belong in `outputs/`; do not commit them.
- `research/` is preserved source material, excluded from installed packages and
  default test discovery. It does not select the production model automatically.
- Track input data, including CSVs. Ignore output directories rather than every
  CSV file; the previous broad ignore rule could hide required model resources.

The package still exposes the established `Vehicle`, `Functions` and `Data`
namespaces. A wholesale namespace migration is unnecessary for this cleanup.
The electrical/planar coupling and approved common parameter set remain separate
model-integration work, documented in [model_baseline.md](model_baseline.md).
