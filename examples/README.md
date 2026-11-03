# Demonstrations

Install `.[dev]` and run from the repository root:

```powershell
python -m examples.battery --duration-s 1
python -m examples.scenarios --help
python -m examples.motor_inverter
python -m examples.skidpad
python -m examples.powertrain
python -m examples.tires
python -m examples.pwm
```

Plotting examples can open windows. Set `MPLBACKEND=Agg` for a headless run.
The skidpad example saves figures under `outputs/`. These demonstrations are
separate from the regression suite: `python -m pytest -q` runs that suite.
