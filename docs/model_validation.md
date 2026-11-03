# Consolidation validation — November 3, 2026

Tested the consolidated model tree on Windows with Python 3.12.14. Exact input
model revisions are in [model_sources.json](model_sources.json).

| Check | Result |
|---|---|
| Default model suite, including new four-motor interface checks | 543 passed, 3 skipped |
| Straight-line scenarios (part throttle, lift, 75 m acceleration, low SOC, hot pack, cold pack) | 6 passed; run as 5 non-lift cases plus a fresh lift case |
| Motor thermal script | All checks passed, including fixed-current rating, time constant, cooling, step size and driven dyno |
| VESC inverter script | All checks passed, including current/DC limits, speed fade, field weakening and vehicle control period |
| Python model and reference source parsing | 72 source files parsed successfully |
| Tejas reference models | AMK CSV map loads (6000 rpm / 10 Nm efficiency 0.89905); SR18 reference car constructs |
| Package boundary | Built vehiclesim 0.2.0 wheel; installed into an isolated directory; imported SR18 and packaged skidpad runner outside the source path; advanced four independent motor commands |
| Patch whitespace | Clean |

The three skips are deliberate zero-slip Dugoff coupling-factor cases. Forces
at zero slip are tested separately; the factor itself is infinite.

The lift-off case was reconciled with the current inverter and wheel-slip model:
it requires zero requested current at lift, less than 0.1 Nm residual motor
torque after 50 ms, and body coasting after 300 ms of wheel-energy release.
The recorded replay and a fresh scenario both passed. The prior test assumed
monotonic body coasting 20 ms after lift despite stored wheel rotational energy.

The model suite's JUnit report and diagnostic replay are generated under
`build/`, which is not committed. Reproduction commands and interpretation limits
are in [model_baseline.md](model_baseline.md).

This validates the assembled source/model APIs. It does not validate the physical
parameter estimates, establish on-car performance, or connect the real SRE-VCU
firmware to the planar electrical car. That coupling remains the next SIL step.
