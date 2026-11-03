# Cleanup verification — November 3, 2026

VehicleSim 0.3.0 reorganizes the consolidated VehSim-1 models into runtime,
tests, examples and archived research. See [repository_layout.md](repository_layout.md)
for path migration and [model_baseline.md](model_baseline.md) for fidelity limits.

## Verified results

| Check | Result |
|---|---|
| `python -m pytest -q` | 569 passed, 3 intentional skips; 572 collected |
| Scope of that one command | Physics tests, all six straight-line scenarios, motor/inverter, thermal, datasheet and state-isolation checks |
| Full-suite duration on this Windows host | 615.19 seconds, about ten minutes |
| New model factories | Independent battery, motor, inverter, chassis, tire and limit state verified |
| Runtime imports | No imports of tests, examples or research from Vehicle runtime |
| Package verification | Fresh 0.3.0 wheel built and installed in isolation; public factories and runtime scenarios imported and advanced without the checkout |
| Wheel contents | Excludes tests, examples, research, retired Dugoff, EMRAX placeholder and old inverter alias |
| Moved examples | Battery run, scenario help and PWM checks passed; headless plotting used Agg |
| Repository layout | Tracked root contains only README, pyproject and gitignore; lowercase test paths verified in Git index for Linux compatibility |
| Documentation and patch | Current local links resolve; patch whitespace check passes |

The three skips are the undefined zero-slip Dugoff coupling factor, with force
behavior at zero slip covered separately. No tests were disabled to complete
the cleanup. Previously script-only checks are now part of default discovery.

Generated wheel, installed smoke-test package and JUnit report live under the
ignored `build/` directory. Research sources and runtime CSV inputs are preserved
under `research/`; the original checkout's unfinished TV merge was not modified.

This verifies organization, packaging and existing modeled behavior. It does not
complete electrical/planar coupling, integrate the real VCU firmware, approve
placeholder parameter values or establish hardware/vehicle performance.
