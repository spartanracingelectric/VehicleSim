# Model baseline and integration boundary

The public model constructors are `Vehicle.create_sr18()` and
`Vehicle.create_skidpad()`. They build independent state from explicit configs.
The older `Vehicle.MainConfigs.SR18.SR18` and `SR18_skidpad` objects remain for
existing callers, but new runs should use factories.

## Electrical model

`create_sr18()` returns four independent AMK motors and VESC inverters sharing
one battery. Commands are signed peak MOTOR current in A, ordered FL/FR/RL/RR.
A scalar broadcasts; a sequence must have four finite values. Convert VCU mA
to A once at the adapter boundary. The battery receives the sum of actual
averaged inverter DC currents, not motor phase currents.

The convenience `torque_request_Nm` input converts motor-shaft torque to iq at
id=0 before inverter limits; field weakening can change the resulting torque.
Setting current and torque requests simultaneously is rejected.

The default inverter current loop runs at 26 kHz. A 10 ms VCU tick contains
260 inverter periods; motor substeps fill those periods exactly. The stand-in
pedal controller is bypassed when external commands are supplied.

The longitudinal chassis still averages motor torques and uses a common motor
speed. It supports longitudinal slip, hydraulic brake/regen blending and an
experimental lateral extension, but it is not the four-corner planar plant.

## Planar model

`create_skidpad()` returns a double-track torque-input plant with four wheel
rotational states, longitudinal/lateral body velocity, yaw, heading and X/Y.
It includes tire forces, quasi-static load transfer, aero and Ackermann steering.
`Vehicle.Simulation.skidpad.runSkidpad` is the packaged research runner.

This plant does not contain the detailed electrical drives or battery model.
Its Python rear-TV controller is a research comparison, not the actual VCU.
An external controller adapter and electrical-to-planar coupling remain open.

## Selected implementations

- Electrical car: `Vehicle.Tire.TireConfig` Dugoff tire and `vesc_srinverter`.
- Planar car: explicitly selected R20 Pacejka tire and planar chassis.
- Steering: one implementation in `Vehicle.Steering.SteeringConfig`; the older
  chassis import is only a compatibility alias.
- BMS: `BMSConfig` provides sampled, latched protection for the electrical car;
  `BMSModel` is a cell-measurement/flag adapter, not another car model.
- Simulation runtime lives in `Vehicle/Simulation`, never in `tests/`.
- Independent study frameworks and retired code live under `research/` and are
  excluded from the installed package and default test discovery.

## Assumptions requiring validation

Model values retain their original source choices. The planar profile uses
277.6 kg including driver, 1.530 m wheelbase, 1.219/1.194 m tracks and 0.2032 m
wheel radius. The electrical profile retains separate placeholder geometry and
mass accounting. These profiles are not interchangeable parameter sets.

Motor heat capacity, fixed coolant temperature, inverter efficiency, VESC Tool
settings, tire track-grip scale, some geometry and controller gains still need
measurement or approval. Motor thermals include copper heating and a fixed
coolant sink; core losses, coupled coolant and thermal shutdown are absent.

The lift-off regression separately checks zero requested current at lift,
less than 0.1 Nm residual motor torque after 50 ms, and body coasting after
300 ms of stored wheel-energy release. These are model criteria, not a target
VCU safety-response deadline.

Use `python -m pytest -q` for all regression checks. Source provenance is in
[model_sources.json](model_sources.json); historical consolidation results are
kept under [history](history/consolidation_validation.md). Passing simulations
alone does not establish target timing, hardware accuracy or on-car performance.
