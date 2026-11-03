# Research and retired source material

This directory is not the installed VehicleSim runtime and does not select the
default SR18 model. It is excluded from the wheel and default pytest discovery.

- `tejas/`: independent acceleration, skidpad and autocross studies, newer tire
  fits and their CSV resources, preserved from Tejas-01. Relative source/resource
  paths are retained. Study-generated plots go to `tejas/Outputs/`, ignored by Git.
- `legacy/`: the original Dugoff implementation, incomplete EMRAX configuration,
  old inverter alias, trace energy script and empty ODE placeholder. These are
  references, not supported alternatives to the active models.

Use the public constructors described in the root README for supported models.
Exact source revisions are recorded in `docs/model_sources.json`.
