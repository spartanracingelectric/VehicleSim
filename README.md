# VehicleSim

A simulation of **SR18's powertrain and straight-line driving**: 4 AMK DD5 hub motors, each with its own inverter,
the 140s3p tractive battery, the BMS, and the car moving down a straight. It's timed the way the real car is timed:

```
driver pedal ─► VCU (10 ms) ─► 4 × inverter (38.5 µs) ─► 4 × motor (4.8 µs steps) ─► gearbox + tires ─► car speed
                                     │                         │
                                     └──── DC current ◄────────┘
                                              ▼
                                   battery pack ─► BMS (50 ms)
```

It's used two ways:
- **On its own**, with a simple stand-in VCU (pedal → current): quick studies of acceleration, power, energy, the
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
python Tests/motorDatasheetTest.py                   # motor + VESC inverter against AMK's datasheet curves
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

car.update(apps_percent=100)                     # one 38.5 µs step, the stand-in VCU decides the current
car.advance(0.01, apps_percent=100)              # 10 ms (260 steps)
car.advance(0.01, current_request_A=76.3)        # 10 ms, current per motor (A peak) decided outside the sim

car.chassis.getSpeed_mps(), car.battery.soc, car.battery.terminal_power_W, car.getTorqueFeedback_Nm()
```

`current_request_A` skips the stand-in VCU, for when something else (the real firmware) decides. It's the VESC's
set-current, which is all SR18's inverters take: there's no torque setpoint.

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
  VCU/                      VCUConfig.py      + Configs/vcu1.py        stand-in VCU: pedal -> current, 10 ms loop
  Inverter/                 InverterConfig.py + Configs/vesc_srinverter.py   the SR custom inverter: VESC firmware's
                                                                             current loop, limits, field weakening, PWM
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

`car.update(...)` simulates **one run of the inverter's current loop (38.5 µs)**: the VESC switches at 13 kHz and
samples in both zero vectors, so its loop runs at 26 kHz. `car.advance(duration_s, ...)` runs as many as fit (260 for
10 ms). Four clocks run inside:

| Clock | Period | What updates |
|---|---|---|
| VCU loop (stand-in) | 10 ms (every 260th update) | reads the pedal, new current request, holds it in between |
| BMS sample | 50 ms (every 1300th update) | reads cells / temperatures / current, sets flags and faults |
| Inverter | 38.5 µs (every update) | current control decides the 3 phase voltages, held for the period |
| Motor | 4.8 µs (8 steps per update) | motor currents, torque, power |

In order (`Vehicle/VehicleConfig.py`):

1. **Request:** the stand-in VCU turns the pedal into a current per motor, unless `current_request_A` was given.
2. **BMS shut down?** Then the contactors are open: motors and inverters off, no current.
3. **Inverter** `updateCurrentControl`, once:
   - field weakening: past 80 % of max duty, push `id` negative (a straight line up to 49.5 A at max duty)
   - the `iq` target is the current request, then the firmware's limits in its order: battery current, motor
     current (the driving side fading out from 16 000 to 20 000 rpm), total current under 148.5 A (105 A rms)
   - PI current controllers + decoupling → `vd`, `vq` → voltage limit (d axis first) → 3 phase voltages
4. **Motor**, 8 steps of 4.8 µs: currents, torque, power. Each step the inverter measures the current and turns the
   motor's power into DC current.
5. **Battery:** draws 4 × the average DC current → voltage sag, SOC, heat.
6. **BMS:** acts on its 50 ms sample.
7. **Chassis:** 4 × torque × gear ratio → wheel force (capped by tire grip) − drag − rolling → new speed, which sets
   the motor speed for the next update.

**Sim time vs computer time:** the sim never looks at the PC's clock. Sim time goes up by exactly one control period per update,
and the VCU and BMS count their loops in sim time, so the timing is exact however long the PC takes.

---

## The models

Each part follows its model PDF (inverter, motor, battery). "Level" is how detailed, per those PDFs.

| Part | Model | Notes |
|---|---|---|
| **VCU (stand-in)** | 10 ms loop, `request = APPS / 100 × 95.6 A` (the current for 21 Nm) | no power limiting, traction control, plausibility or regen: those come from the real firmware (SIL) |
| **Inverter** | VESC firmware's FOC loop. Level 0 power (flat 97 %), Level 1 average voltage (SVM, `0.95 · V_bus / √3`); Level 2 PWM switching only in `pwmTest` | current setpoint only, limits in the firmware's order, PI (Kp 0.18 V/A, Ki 67.5 V/As) with cross + back-EMF decoupling, half-period angle lead, averaged current sensing, duty-cycle field weakening down to −49.5 A, d axis first at the voltage limit |
| **Motor** | Level 2 average-value d/q model, forward Euler at 5 µs | `Te = 1.5·(poles/2)·[λf·iq + (Ld−Lq)·id·iq]`, copper loss; speed set by the car |
| **Battery** | Level 2 Thevenin: OCV(SOC) − I·R | 140s3p, 5 Ah / 6.7 mΩ cells, coulomb counting, I²R heat, no cooling |
| **BMS** | samples every 50 ms | latched shutdown on cell over/undervoltage (4200 / 2800 mV) or temperature (60 / 0 °C); **> 80 kW is only flagged** |
| **Chassis** | straight line: wheel force (capped at μ·weight) − drag − rolling | no wheel slip, load transfer, brake force, gearbox loss or rotating inertia |

Motor equations:
```
did/dt = (vd − Rs·id + ωe·Lq·iq) / Ld
diq/dt = (vq − Rs·iq − ωe·(Ld·id + λf)) / Lq
Pe     = 1.5 · (vd·id + vq·iq)          P_copper = 1.5 · Rs · (id² + iq²)
```

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
| field weakening current | 35 A rms | datasheet "magn. current Im", used as the VESC's `fw_current_max` |
| VCU loop / BMS sample | 10 ms / 50 ms | team |
| BMS limits | 4200 / 2800 mV, 60 / 0 °C, 80 kW | `bms1.py` |
| battery | 140s3p, 5 Ah, 6.7 mΩ cells, 50.8 kg | `Battery/Configs/sr17.py` |
| tire radius / grip | 0.2032 m / μ 1.2 | `tire1.py` |
| vehicle mass | 204.1 kg + battery | `SR18.py` |
| **inverter efficiency** | 0.97 | **placeholder** |
| inverter timing and fixed limits | 13 kHz switching, 26 kHz current loop, 60 A battery per inverter, max duty 0.95, 100k erpm fading from 80 % | compiled-in defaults: `hw_SRINVERTER.h` + `mcconf_default.h` in SR-Custom-Inverter-FW |
| **inverter settings** | ±148.5 A motor current, Kp 0.18 / Ki 67.5, cross + back-EMF decoupling, 49.5 A field weakening | **what VESC Tool would need for the AMK**, swap for what gets flashed (firmware defaults: ±60 A, Kp 0.03 / Ki 50, no decoupling, no field weakening) |
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
- **Checks** (`checks.py`): `torqueAbout`, `torqueNeverAbove`, `currentNeverAbove`, `accelerates`, `coasts`, `staysParked`,
  `reachesDistance`, `reachesSpeed`, `noOverpower`, `overpowerFlagged`, `noShutdown`, `shutsDown`. `ALWAYS` runs on
  every scenario: motor current never over the inverter's limit, the VCU only changes on its loop, the BMS flag matches
  its reading.

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
| `motorDatasheetTest.py` | motor + VESC inverter against AMK's torque-speed curves at 500 / 600 V |
| `motorInverterTest.py` | one motor + inverter on a dyno: current steps with the power balance, voltage limit with field weakening |
| `vescInverterTest.py` | the firmware's settings on a dyno: current and battery current limits, speed fade, field weakening, decoupling, 26 kHz car |
| `inverterTest.py` | inverter math: power conversion, transforms, voltage limit (run it after touching the inverter) |
| `pwmTest.py` | Level 2: real IGBT switching into a test load, against the average model |
| `powertrainTest.py` | 15 s full throttle with the full plot, and a low-pack shutdown |
| `pidTest.py`, `tireTest.py` | the PID on its own; the tire model plot |

---

## Results so far

| | Result |
|---|---|
| 75 m accel event | 3.99 s, 126 km/h (grip-limited, no driver mass) |
| 0–100 km/h | 2.82 s |
| top speed | 127 km/h (19 550 rpm: the VESC fades the current out from 16 000 rpm) |
| peak pack power | ~158 kW (no power limiting in the sim; the BMS flags > 80 kW from ~1.6 s) |
| peak torque per motor | 24.6 Nm: the VESC holds 95.6 A and field weakening's negative id adds reluctance torque |
| motor + VESC vs AMK datasheet | +8 to +31 % up to ~17 000 rpm (field weakening from 80 % duty), then well under (speed fade) |
| full throttle until empty | 11 min 30 s, 24.4 km, 7.07 kWh, SOC 100 % → 3.2 %, pack 25 → 52.7 °C (no cooling), ends in undervoltage (from before the VESC inverter, not re-run) |

These are with the stand-in VCU (full pedal = 95.6 A, 21 Nm at id = 0). Driven by the real firmware, full pedal is 76.3 A per motor
(≈ 16.8 Nm), so launches are slower.

**How much to trust it:** good from 0 to ~95 km/h (the main errors are the placeholders: driver mass, drag);
reasonable from ~95 to 127 km/h with field weakening (the VESC's settings there are guesses until the flashed ones are in). Good for
testing VCU logic and timing and for comparing options; not yet for absolute lap times or energy per lap.

---

## Design decisions

- **The 80 kW rule is only flagged, never enforced.** The flashed VCU has its own power limiting; a limiter here
  would duplicate or misrepresent it.
- **One motor + inverter simulated, × 4.** All four get the same request, so they'd be identical. Has to change when
  they get different ones (traction control, torque vectoring).
- **The inverter is the VESC, only.** SR18's inverters run VESC firmware, so `InverterConfig` follows its FOC loop
  (`mcpwm_foc.c`): a current setpoint (no torque setpoint), limits in the firmware's order (field weakening, battery
  current via mod_q, motor current with the speed fade, total), field weakening as a straight line on the filtered
  duty cycle, SVM only. Its defaults are the firmware's compiled-in ones; `vesc_srinverter.py` sets what VESC Tool
  would. The stand-in VCU asks for current like the real firmware does.
- **Torque names match the log:** command (what the inverter's current target makes, *MCM Commanded Torque*),
  feedback (what the motor made, *MCM Torque Feedback*).
- **Averaged current sensing.** Sampling once per period let the current drift off its target at high speed (the
  rotor turns ~23° per 38.5 µs at 20 000 rpm); averaging over the period fixes it.
- **Average inverter model, not switching.** Switching needs 0.5 µs steps (10× slower) and `pwmTest` showed the
  averages match within 2.5 V.
- **The Crows log is SR17 (EMRAX, Cascadia inverter, ~340 V)**, so it can't validate SR18's numbers. Its pedal and
  speed traces can still be used as inputs.

---

## Known gaps

**Placeholders (quick, they change every result):** driver mass, drag area, rolling resistance, inverter efficiency
and the VESC Tool settings (current limits, gains, decoupling, field weakening), EMRAX electrical values (for an
SR17 config).

**Not modeled yet:**
- VESC: the 180 A absolute-current fault, field weakening backoff and ramp, MTPA, integrator clamping (the sim
  stops integrating instead)
- brake force and regen, wheel slip and load transfer, steering / cornering (straight line only)
- 4 separate motors / inverters
- battery current limit, cooling, RC voltage recovery; motor / inverter temperatures (the 105 A max is allowed for
  1.24 s, not enforced)
- inverter conduction and switching losses (flat 3 % now)
- a driver model and laps; replaying pedals from a log

**Model notes:** the VESC's battery current limit only counts `mod_q · iq` (like the firmware, see its TODO), so with
field weakening the real battery current goes past it (~86 A per inverter at full throttle vs the 60 A limit);
started cold above ~18 000 rpm at 500 V the currents run away before field weakening builds up (the car never gets
there cold, the datasheet test spins up); the Ld/Lq swap is a judgment call; a 0 → 100 % pedal step overshoots ~11 % for ~2.5 ms (realistic ramps don't).

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
