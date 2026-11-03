# VehicleSim

A simulation of **SR18's powertrain and straight-line driving**: 4 AMK DD5 hub motors, each with its own inverter,
the 140s3p tractive battery, the BMS, and the car moving down a straight. It's timed the way the real car is timed:

```
driver pedal ─► VCU (10 ms) ─► 4 × inverter (100 µs) ─► 4 × motor (5 µs steps) ─► gearbox + tires ─► car speed
                                     │                         │
                                     └──── DC current ◄────────┘
                                              ▼
                                   battery pack ─► BMS (50 ms)
```

It's used two ways:
- **On its own**, with a simple stand-in VCU (pedal → torque): quick studies of acceleration, power, energy, the
  battery and the motors. Tests in `Tests/`.
- **Driven by the real VCU firmware** (software-in-the-loop, in the SRE-VCU repo): the flashed C code runs on the PC
  in lockstep with this sim, and its motor current commands drive the car.

---

## Contents
1. [Quick start](#quick-start)
2. [Using it](#using-it)
3. [Project layout](#project-layout)
4. [How the car is put together](#how-the-car-is-put-together)
5. [What happens in one update](#what-happens-in-one-update)
6. [The models](#the-models)
7. [Parameters and where they come from](#parameters-and-where-they-come-from)
8. [Testing](#testing)
9. [Results so far](#results-so-far)
10. [Design decisions](#design-decisions)
11. [Known gaps](#known-gaps)
12. [Glossary](#glossary)

---

## Quick start

**Needs:** Python 3.10+ with `numpy`, plus `pandas`, `matplotlib` and `pytest` for the logs, plots and tests:

```bash
pip install -e ".[dev]"
```

> **VS Code:** pick the Python that has those packages (`Ctrl+Shift+P` → *Python: Select Interpreter*).
> On the team PC that's the Microsoft Store Python 3.12, *not* `C:\msys64\mingw64\bin\python.exe`.

Run from the repo root (every test also works as `python -m Tests.<name>`):

```bash
pytest Tests/scenarioTest.py -v                      # the straight-line scenarios, pass/fail each (~20 s)
python Tests/scenarioTest.py "lift off" --plot       # one scenario, plotted
python Tests/powertrainTest.py                       # 15 s full-throttle run + plot (~30 s)
python Tests/motorDatasheetTest.py                   # motor + inverter against AMK's datasheet curves
```

The sim runs about **2× slower than real time** (15 s of driving ≈ 30 s on the PC). It has its own clock, so
that doesn't change any result.

---

## Using it

### From Python

```python
from copy import deepcopy
from Vehicle.MainConfigs.SR18 import SR18

car = deepcopy(SR18)                             # always a copy: SR18 carries run state
car.battery.reset(initial_soc=0.6)               # start from 60 % (sets the pack voltage to match)
car.chassis.speed_mps = 20.0                     # rolling start

car.update(apps_percent=100)                     # one 100 µs step, the stand-in VCU decides the torque
car.advance(0.01, apps_percent=100)              # 10 ms (100 steps)
car.advance(0.01, torque_request_Nm=16.8)        # 10 ms, torque per motor decided outside the sim
car.advance(0.01, current_request_A=76.3)        # 10 ms, current per motor, like a VESC's set-current

car.chassis.getSpeed_mps(), car.battery.soc, car.battery.terminal_power_W, car.getTorqueFeedback_Nm()
```

`torque_request_Nm` / `current_request_A` skip the stand-in VCU, for when something else (the real firmware)
decides. `current_request_A` is what SR18's VESC inverters actually get.

### From another repo (SRE-VCU's SIL tests)

VehicleSim installs as a package (`pyproject.toml`): `Vehicle`, `Functions` and `Data`, with the same imports.

```bash
pip install "vehiclesim @ git+https://github.com/spartanracingelectric/VehicleSim.git@VehSim-1"   # from GitHub
pip install -e path/to/VehicleSim                                                                # local checkout
```

SRE-VCU's SIL container installs it from GitHub (the `VehSim-1` branch) every time it starts, so **changes here
reach the SIL tests once they're pushed**. There, `sre_test.vehicle.run()` runs the real firmware one 10 ms loop at a
time: the car's state goes to the firmware as VESC / BMS CAN frames, and the motor current commands it sends drive
the car with `advance(0.01, current_request_A=...)`. See `SRE-VCU/dev/sil/tests/README.md`.

---

## Project layout

```
Vehicle/
  VehicleConfig.py          the car: holds every part, runs update() / advance() of the whole chain
  MainConfigs/SR18.py       SR18 assembled from the parts below (swap parts here)
  VCU/                      VCUConfig.py      + Configs/vcu1.py        stand-in VCU: pedal -> torque, 10 ms loop
  Inverter/                 InverterConfig.py + Configs/inverter1.py   current control, field weakening, PWM
  Motor/                    MotorConfig.py    + Configs/amk_dd5.py, emrax_228.py (placeholder), AMK datasheet
  Battery/                  TractiveBatteryPack.py, Cell.py + Configs/sr17.py (SR18's 140s3p pack), battery2/3
  BMS/                      BMSConfig.py      + Configs/bms1.py        cell monitoring, faults, 80 kW flag
  Chassis/                  ChassisConfig.py  + Configs/chassis1.py    wheel force, grip limit, drag, rolling
  Tire/                     TireConfig2.py    + Configs/tire1.py       radius, grip (Dugoff slip model not used yet)
  Axel/ Brake/ Driver/      placeholders (mass only)

Functions/                  clarkePark.py (3-phase <-> rotor frame), PIDcontroller.py (PID with anti-windup),
                            loadConfigs.py (loads every config in a Configs/ folder), constants.py
Tests/                      scenario tests + model tests, see Testing
Data/                       autox80kwh.csv (MoTeC log, Crows autocross, SR17: the EMRAX car), csvParser.py,
                            channels.py (log channel names), plot.py (all plotting, no calculations)
battery_sim.py, simrunner.py   older stand-alone scripts, not part of the powertrain chain
```

Note: the car is SR18, but its battery config file is still called `sr17.py` (the 140s3p pack).

---

## How the car is put together

Every part follows the same pattern:

| | Example | Holds |
|---|---|---|
| **Class** | `Vehicle/Motor/MotorConfig.py` | the part's physics: its own `update()` and getters |
| **Config** | `Vehicle/Motor/Configs/amk_dd5.py` | one specific part: `amk_dd5 = MotorConfig(Rs=..., ...)` |
| **Car** | `Vehicle/MainConfigs/SR18.py` | which configs make up the car, plus car-level numbers (mass, gear ratio, motor count) |

Rules for config files (`loadConfigs` imports every `.py` in a `Configs/` folder):
1. **The variable has the same name as the file.** `amk_dd5.py` defines `amk_dd5`.
2. **No code that runs on import** (no prints, test calls or `plt.show()`). Put demos under
   `if __name__ == "__main__":`. A config that does work on import runs every time *any* sim starts.

**Adding a part or a variant:** a new config file in that part's `Configs/` (e.g. `Motor/Configs/emrax_228.py`),
then point the car at it in `MainConfigs/`. **Another car:** a new file in `MainConfigs/`, used with
`Scenario(vehicle=...)` in the tests or imported directly.

**Always `deepcopy` the car before running it.** `SR18` is one shared object and carries run state (battery
charge, BMS latch, motor currents, controller memory). The test helpers do it for you.

---

## What happens in one update

`car.update(...)` simulates **one inverter switching period (100 µs)**; `car.advance(duration_s, ...)` runs as many
as fit (100 for 10 ms). Four clocks run inside:

| Clock | Period | What updates |
|---|---|---|
| VCU loop (stand-in) | 10 ms (every 100th update) | reads the pedal, new torque request, holds it in between |
| BMS sample | 50 ms (every 500th update) | reads cells / temperatures / current, sets flags and faults |
| Inverter | 100 µs (every update) | current control decides the 3 phase voltages, held for the period |
| Motor | 5 µs (20 steps per update) | motor currents, torque, power |

In order (`Vehicle/VehicleConfig.py`):

1. **Request:** the stand-in VCU turns the pedal into a torque per motor, unless `torque_request_Nm` or
   `current_request_A` was given.
2. **BMS shut down?** Then the contactors are open: motors and inverters off, no current.
3. **Inverter** `updateCurrentControl`, once:
   - field weakening: if the voltage is running out, push `id` negative
   - the `iq` target: from the torque (capped at 21 Nm, reluctance torque counted), or the current request directly
   - speed limit fade near 20 000 rpm, total current capped at 105 A rms
   - PI current controllers + feedforward → `vd`, `vq` → voltage limit → 3 phase voltages
4. **Motor**, 20 steps of 5 µs: currents, torque, power. Each step the inverter measures the current and turns the
   motor's power into DC current.
5. **Battery:** draws 4 × the average DC current → voltage sag, SOC, heat.
6. **BMS:** acts on its 50 ms sample.
7. **Chassis:** 4 × torque × gear ratio → wheel force (capped by tire grip) − drag − rolling → new speed, which sets
   the motor speed for the next update.

**Sim time vs computer time:** the sim never looks at the PC's clock. Sim time goes up by exactly 100 µs per update,
and the VCU and BMS count their loops in sim time, so the timing is exact however long the PC takes.

---

## The models

Each part follows its model PDF (inverter, motor, battery). "Level" is how detailed, per those PDFs.

| Part | Model | Notes |
|---|---|---|
| **VCU (stand-in)** | 10 ms loop, `request = APPS / 100 × 21 Nm` | no power limiting, traction control, plausibility or regen: those come from the real firmware (SIL) |
| **Inverter** | Level 0 power (flat 97 %), Level 1 average voltage (SVM, `V_bus / √3`); Level 2 PWM switching only in `pwmTest` | AMK's controller gains (Kp 0.58/0.64 V/A, Tn 1.2 ms), feedforward, averaged current sensing, field weakening PI (Kp 0.08 A/V, Tn 6 ms) down to −35 A rms, d axis first at the voltage limit, torque or current setpoint |
| **Motor** | Level 2 average-value d/q model, forward Euler at 5 µs | `Te = 1.5·(poles/2)·[λf·iq + (Ld−Lq)·id·iq]`, copper loss; speed set by the car; one-node winding temperature |
| **Battery** | Level 2 Thevenin: OCV(SOC) − I·R | 140s3p, 5 Ah / 6.7 mΩ cells, coulomb counting, I²R heat, no cooling |
| **BMS** | samples every 50 ms | latched shutdown on cell over/undervoltage (4200 / 2800 mV) or temperature (60 / 0 °C); **> 80 kW is only flagged** |
| **Chassis** | straight line: wheel force (capped at μ·weight) − drag − rolling | no wheel slip, load transfer, brake force, gearbox loss or rotating inertia |

Motor equations:
```
did/dt = (vd − Rs·id + ωe·Lq·iq) / Ld
diq/dt = (vq − Rs·iq − ωe·(Ld·id + λf)) / Lq
Pe     = 1.5 · (vd·id + vq·iq)          P_copper = 1.5 · Rs · (id² + iq²)
Cth · dT/dt = P_copper − (T − T_coolant) / Rth      (average winding temperature, solved exactly each step)
```
The winding temperature (`motor.getWindingTemp_C()`, logged as `motor_temp_C` in scenarios) is one node: no
hotspot, no core loss, and nothing uses it yet (no derating, Rs and λf don't change with it). A motor config
without `thermalResistance_KperW` / `heatCapacity_JperK` stays at the coolant temperature.

Power flow, checked to balance in the tests (motor power = shaft + copper, within 1 %):
`battery chemical → − battery I²R → pack terminals → − inverter 3 % → motors → − copper → shafts`.

---

## Parameters and where they come from

| Parameter | Value | Source |
|---|---|---|
| motors / gear ratio | 4 / 11.81 | team |
| motor max torque / speed / current | 21 Nm / 20 000 rpm / 105 A rms (1.24 s) | AMK datasheet |
| motor Rs (per phase) | 0.0675 Ω | datasheet Rtt 0.135 Ω is terminal-to-terminal, ÷ 2 |
| motor λf | 0.0293 Wb | from ke = 18.8 V/1000 rpm (Kt would give 0.0245, 16 % too low) |
| motor Lq / Ld | 0.24 / 0.12 mH | **swapped vs the datasheet labels**, only this way fits its curves |
| field weakening current | 35 A rms | datasheet "magn. current Im" |
| motor Rth (windings → coolant) | 0.175 K/W | datasheet "S1 dT=100K": 100 K over the coolant at 53.1 A rms (571 W copper) |
| **motor heat capacity** | 400 J/K (70 s time constant) | **placeholder**, fit from a dyno heat-up or a KTY84 log |
| motor coolant temperature | 25 °C | inlet, fixed (no cooling loop model) |
| controller gains | current Kp 0.58/0.64, Tn 1.2 ms; voltage Kp 0.08, Tn 6 ms | datasheet controller settings |
| VCU loop / BMS sample | 10 ms / 50 ms | team |
| BMS limits | 4200 / 2800 mV, 60 / 0 °C, 80 kW | `bms1.py` |
| battery | 140s3p, 5 Ah, 6.7 mΩ cells, 50.8 kg | `Battery/Configs/sr17.py` |
| tire radius / grip | 0.2032 m / μ 1.2 | `tire1.py` |
| vehicle mass | 204.1 kg + battery | `SR18.py` |
| **inverter efficiency** | 0.97 | **placeholder** |
| **switching frequency** | 10 kHz | **placeholder** |
| **drag area / rolling resistance** | 1.2 m² / 0.015 | **placeholders** (`chassis1.py`) |
| **driver mass** | not included | **placeholder** (`driver1` is 10 kg) |

---

## Testing

### Scenarios: the easy way to add a test
Each test is one row in `SCENARIOS` in `Tests/scenarioTest.py`:

```python
Scenario("half throttle from 60 km/h",
         pedal=hold(50), run_time_s=2.0, start_speed_kph=60,
         checks=[torqueAbout(10.5, after_s=0.05), accelerates(), noShutdown()]),
```

- **Options:** `pedal`, `checks`, `run_time_s`, `stop_at_distance_m`, `stop_when` (e.g. `stopWhenBmsShutsDown`),
  `start_soc`, `start_temp_C`, `start_speed_kph`, `record_every_s` (log less often on long runs), `vehicle`.
- **Pedal shapes** (`pedals.py`): `hold(30)`, `ramp(to=100, over_s=0.8)`, `steps((0, 100), (2.0, 0))`,
  `pulses(100, on_s, off_s)`.
- **Checks** (`checks.py`): `torqueAbout`, `torqueNeverAbove`, `accelerates`, `coasts`, `staysParked`,
  `reachesDistance`, `reachesSpeed`, `noOverpower`, `overpowerFlagged`, `noShutdown`, `shutsDown`. `ALWAYS` runs on
  every scenario: torque never over the motor max, the VCU only changes on its loop, the BMS flag matches its reading.

```bash
pytest Tests/scenarioTest.py -v                         # every scenario
pytest Tests/scenarioTest.py -k lift                    # the ones with "lift" in the name
python Tests/scenarioTest.py --plot                     # run all, plot each (--save plots: PNGs instead)
python Tests/scenarioTest.py --slow                     # also the long ones (full throttle until the pack is empty)
```

A failure says what went wrong, e.g. `torque feedback reached 6.30 Nm, expected 10.0 Nm +/-0.25`.

### Model tests
Run with `python Tests/<name>.py` (plots open a window; `MPLBACKEND=Agg` to skip them):

| Test | Checks |
|---|---|
| `motorDatasheetTest.py` | motor + inverter against AMK's torque-speed curves at 500 / 600 V |
| `motorInverterTest.py` | one motor + inverter on a dyno: torque steps, voltage limit with field weakening, current setpoint steps |
| `motorThermalTest.py` | winding temperature: datasheet S1 rating (100 K at 53.1 A rms), time constant, cool-down, step size, heating on a dyno |
| `inverterTest.py` | inverter math: power conversion, transforms, voltage limit (run it after touching the inverter) |
| `pwmTest.py` | Level 2: real IGBT switching into a test load, against the average model |
| `powertrainTest.py` | 15 s full throttle with the full plot, and a low-pack shutdown |
| `pidTest.py`, `tireTest.py` | the PID on its own; the tire model plot |

---

## Results so far

| | Result |
|---|---|
| 75 m accel event | 3.98 s, 129 km/h (grip-limited, no driver mass) |
| 0–100 km/h | 2.82 s |
| top speed | 129 km/h (motors at the 20 000 rpm limit) |
| peak pack power | ~151 kW (no power limiting in the sim; the BMS flags > 80 kW from ~1.6 s) |
| motor vs AMK datasheet | exact up to the knee, within −9 % / +19 % past it with field weakening |
| full throttle until empty | 11 min 30 s, 24.4 km, 7.07 kWh, SOC 100 % → 3.2 %, pack 25 → 52.7 °C (no cooling), ends in undervoltage |
| motor winding temperature | accel event 25 → 31.7 °C (placeholder heat capacity) |

These are with the stand-in VCU (full pedal = 21 Nm). Driven by the real firmware, full pedal is 76.3 A per motor
(≈ 16.8 Nm), so launches are slower.

**How much to trust it:** good from 0 to ~95 km/h (the main errors are the placeholders: driver mass, drag);
reasonable from ~95 to 129 km/h with field weakening (within ~20 % of the datasheet, slightly optimistic). Good for
testing VCU logic and timing and for comparing options; not yet for absolute lap times or energy per lap.

---

## Design decisions

- **The 80 kW rule is only flagged, never enforced.** The flashed VCU has its own power limiting; a limiter here
  would duplicate or misrepresent it.
- **One motor + inverter simulated, × 4.** All four get the same request, so they'd be identical. Has to change when
  they get different ones (traction control, torque vectoring).
- **Torque or current setpoint.** The stand-in VCU asks for torque (like the AMK inverter); the real firmware asks
  for current (like a VESC). Below field weakening they give the same result; with it, the current setpoint holds
  the current and the torque is whatever it makes, as a VESC does.
- **Torque names match the log:** request (VCU, *MCM Torque Command*), command (inverter after its limits,
  *MCM Commanded Torque*), feedback (what the motor made, *MCM Torque Feedback*).
- **Averaged current sensing.** Sampling once per period let the current drift off its target at high speed (the
  rotor turns ~60° per 100 µs at 20 000 rpm); averaging over the period fixes it.
- **Average inverter model, not switching.** Switching needs 0.5 µs steps (10× slower) and `pwmTest` showed the
  averages match within 2.5 V.
- **The Crows log is SR17 (EMRAX, Cascadia inverter, ~340 V)**, so it can't validate SR18's numbers. Its pedal and
  speed traces can still be used as inputs.

---

## Known gaps

**Placeholders (quick, they change every result):** driver mass, drag area, rolling resistance, inverter efficiency
and switching frequency, EMRAX electrical values (for an SR17 config), motor heat capacity.

**Not modeled yet:**
- brake force and regen, wheel slip and load transfer, steering / cornering (straight line only)
- 4 separate motors / inverters
- battery current limit, cooling, RC voltage recovery; inverter temperature; anything using the motor temperature
  (derating, Rs / λf vs temperature, the KTY84 sensor), its hotspot and core loss (the 105 A max is allowed for
  1.24 s, not enforced: the one-node average only rises ~7 K in that time)
- inverter conduction and switching losses (flat 3 % now)
- a driver model and laps; replaying pedals from a log

**Model notes:** field weakening gives up to ~19 % more torque than AMK's curves mid-range; the Ld/Lq swap is a
judgment call; a 0 → 100 % pedal step overshoots ~11 % for ~2.5 ms (realistic ramps don't).

**Validation:** compare against SR18 logs once it runs (same pedal input; compare speed, pack current, voltage).

---

## Glossary

| Term | Meaning |
|---|---|
| APPS / BPS | accelerator pedal position / brake pressure sensors |
| SOC / OCV | state of charge / open-circuit voltage (no-load cell voltage) |
| V_bus | DC bus voltage = pack terminal voltage |
| d / q axes | rotor frame: d along the magnets (flux), q 90° ahead (torque) |
| Clarke / Park | 3-phase ↔ 2-axis stationary ↔ rotating rotor frame transforms |
| ωm / ωe | shaft speed / electrical speed (= 5 pole pairs × ωm) |
| back-EMF | voltage the spinning magnets generate, `ωe × λd`, grows with speed |
| field weakening | negative `id` that cancels part of the magnet flux so back-EMF drops at high speed |
| SVM | space vector modulation, gets V_bus/√3 out of the inverter (15 % more than plain sine PWM) |
| anti-windup | stop a PI's integral growing while its output is saturated |
| SIL / HIL | software- / hardware-in-the-loop |
