# VehicleSim

A powertrain and straight-line vehicle simulation of **SR17 with 4 AMK DD5 hub motors and 4 inverters**.

It simulates the whole chain, timed the way the real car is timed:

```
driver pedal ─► VCU (10 ms) ─► 4 × inverter (100 µs) ─► 4 × motor (5 µs steps) ─► gearbox + tires ─► car speed
                                     │                         │
                                     └──── DC current ◄────────┘
                                              ▼
                                   battery pack ─► BMS (50 ms)
```

It's built so the **real VCU firmware can be plugged in later** (software-in-the-loop, see the end).

---

## Contents
1. [Quick start](#quick-start)
2. [Project layout](#project-layout)
3. [How the car is put together](#how-the-car-is-put-together)
4. [What happens in one update](#what-happens-in-one-update)
5. [The models](#the-models)
6. [Parameters and where they come from](#parameters-and-where-they-come-from)
7. [Testing](#testing)
8. [Results so far](#results-so-far)
9. [Design decisions](#design-decisions)
10. [Known gaps and next steps](#known-gaps-and-next-steps)
11. [Connecting the real VCU firmware](#connecting-the-real-vcu-firmware-software-in-the-loop)
12. [Glossary](#glossary)

---

## Quick start

**Needs:** Python 3.12 with `numpy`, `matplotlib`, `pandas` (and `pytest` for the scenario tests).

> **VS Code:** pick the Python that has those packages (`Ctrl+Shift+P` → *Python: Select Interpreter*).
> On this machine that's the Microsoft Store Python 3.12, *not* `C:\msys64\mingw64\bin\python.exe`.

Run from the repo root (every test also works as `python -m Tests.<name>`):

```bash
python Tests/powertrainTest.py                       # full car: 15 s full-throttle run + plot (~30 s)
pytest Tests/scenarioTest.py -v                      # all straight-line scenarios, pass/fail each (~20 s)
python Tests/scenarioTest.py "lift off" --plot       # run one scenario and plot it
python Tests/motorDatasheetTest.py                   # motor + inverter vs the AMK datasheet curves
```

The sim runs about **2× slower than real time** (15 s of driving ≈ 30 s on the computer).
That's fine: the sim has its own clock (see [sim time vs computer time](#sim-time-vs-computer-time)).

---

## Project layout

```
Vehicle/
  VehicleConfig.py          the car: holds every part and runs one update of the whole chain
  MainConfigs/SR17.py       SR17 assembled from the parts below (swap parts here)
  VCU/                      VCUConfig.py   + Configs/vcu1.py       pedal -> torque request, 10 ms loop
  Inverter/                 InverterConfig.py + Configs/inverter1.py   current control, field weakening, PWM
  Motor/                    MotorConfig.py + Configs/amk_dd5.py, emrax_228.py (placeholder), amk_motor_datasheet.pdf
  Battery/                  TractiveBatteryPack.py, Cell.py + Configs/sr17.py (+ battery2/3)
  BMS/                      BMSConfig.py   + Configs/bms1.py       cell monitoring, faults, 80 kW flag
  Tire/                     TireConfig2.py + Configs/tire1.py
  Axel/ Brake/ Driver/      placeholders (mass only for now)

Functions/
  clarkePark.py             Clarke / Park transforms (3-phase <-> rotor frame)
  PIDcontroller.py          reusable PID with anti-windup (used by the inverter, can be used anywhere)
  loadConfigs.py            loads every config in a Configs/ folder
  constants.py              g, air density, unit conversions

Tests/
  scenarioTest.py           THE scenario table: add a row to add a test
  scenario.py               Scenario + runScenario (the sim loop, written once) + plotScenario
  pedals.py                 pedal shapes: hold, ramp, steps, pulses
  checks.py                 reusable pass/fail rules + ALWAYS (checked on every scenario)
  powertrainTest.py         15 s acceleration with the full plot
  motorDatasheetTest.py     motor + inverter vs AMK's torque-speed curves
  motorInverterTest.py      one motor + inverter on a dyno: torque steps, voltage limit
  inverterTest.py           inverter math (power conversion, transforms, voltage limit)
  pwmTest.py                inverter Level 2: real IGBT switching into a test load
  pidTest.py                the PID on its own
  tireTest.py               tire model plot (teammate's script)

Data/
  autox80kwh.csv            MoTeC log, autocross at Crows (the older EMRAX car!)
  csvParser.py, channels.py MoTeC loading and channel names
  plot.py                   all plotting (no calculations in here)

battery_sim.py, simrunner.py   older stand-alone scripts, not part of the powertrain chain
```

---

## How the car is put together

Every part follows the same pattern:

| | Example | Holds |
|---|---|---|
| **Class** | `Vehicle/Motor/MotorConfig.py` | the part's physics: its own `update()` and getters |
| **Config** | `Vehicle/Motor/Configs/amk_dd5.py` | one specific part: `amk_dd5 = MotorConfig(Rs=..., ...)` |
| **Car** | `Vehicle/MainConfigs/SR17.py` | which configs make up the car, plus car-level numbers (mass, gear ratio, motor count) |

Two rules for config files (`loadConfigs` imports every `.py` in a `Configs/` folder):

1. **The variable must have the same name as the file.** `amk_dd5.py` must define `amk_dd5`.
2. **No code that runs on import** (no prints, no test calls, no `plt.show()`). Put demos under
   `if __name__ == "__main__":`. A config that does work on import runs every time *any* sim starts.

**To try another car**, make another file in `MainConfigs/` and swap it in (`VEHICLE = SR17` in a test,
or `Scenario(vehicle=...)`). `emrax_228.py` is waiting for the EMRAX's electrical values.

**Always `deepcopy` the car before running it.** `SR17` is one shared object and it carries run state
(battery charge, BMS latch, motor currents, controller memory). The test helpers already do this.

---

## What happens in one update

`vehicle.update(apps_percent)` simulates **one inverter switching period (100 µs)**. Four clocks run inside it:

| Clock | Period | What updates |
|---|---|---|
| VCU loop | 10 ms (every 100th update) | reads the pedal, new torque request, holds it in between |
| BMS sample | 50 ms (every 500th update) | reads cells / temps / current, sets flags and faults, holds in between |
| Inverter | 100 µs (every update) | current controller decides 3 phase voltages, held for the period |
| Motor | 5 µs (20 steps per update) | motor currents, torque, power |

In order (`Vehicle/VehicleConfig.py`):

1. **VCU**: `vcu.update(pedal)` → torque request per motor (changes only on its 10 ms loop).
2. **BMS shut down?** If so, contactors are open: motors and inverters off (`turnOff()`), no current.
3. **Inverter** `updateCurrentControl(...)`, once:
   - cap torque at the motor max (21 Nm) and fade it out near the 20000 rpm speed limit
   - field weakening: if the voltage is running out, push `id` negative
   - torque → current target `iq*`, capped by the 105 A rms current limit
   - PI current controllers + feedforward → `vd`, `vq` → voltage limit → 3 phase voltages
4. **Motor**, 20 steps of 5 µs: `motor.update_from_phase_voltages(...)` → currents, torque, power.
   Each step the inverter also measures the current (`measureCurrents`) and turns the motor's power into
   DC current (`updatePower`).
5. **Battery**: `battery.update(4 × average DC current)` → voltage sag, SOC, heat.
6. **BMS**: `bms.updateFromPack(...)` (only acts on its 50 ms sample).
7. **Chassis**: motor torque × 4 × gear ratio → wheel force (capped by tire grip) − drag − rolling →
   new car speed → next update's motor speed.

### Sim time vs computer time
The sim never looks at the computer's clock. Sim time is a number that goes up by exactly 100 µs per update,
and the VCU and BMS count their loops in sim time. So the timing is exact however long the computer takes,
and you could pause the sim and the results wouldn't change. Only *hardware*-in-the-loop would need real time.

---

## The models

Each part follows its model PDF (inverter, motor, battery). Level = how detailed, per those PDFs.

### VCU (stand-in) — `Vehicle/VCU/VCUConfig.py`
- 10 ms loop with a countdown timer in sim time; holds its request between loops
- pedal map: `request = clamp(APPS, 0, 100) / 100 × 21 Nm` (straight line placeholder)
- **does not** model the real firmware's power limiting, traction control, plausibility checks or regen.
  That's on purpose: those will come from the real firmware (see SIL).

### Inverter — `Vehicle/Inverter/InverterConfig.py`
| Level | Method | Used by the car? |
|---|---|---|
| 0: constant efficiency | `updatePower`: `P_dc = P_ac / 0.97` (motoring) or `× 0.97` (regen), `I_dc = P_dc / V_bus` | yes, every motor step |
| 1: average voltage | `updateVoltage`: d/q command → inverse Park → voltage limit → inverse Clarke → SVM offset → duty cycles | yes |
| 2: PWM switching | `updateSwitching`: carrier comparison, complementary IGBT gates | only in `pwmTest` (10× slower, results ≈ the same) |

Current control (`updateCurrentControl`, once per 100 µs):
- two PI controllers (`PIDController`, AMK's gains: Kp 0.58/0.64 V/A, Tn 1.2 ms) + feedforward that
  cancels the motor's speed terms (`−ωe·Lq·iq` and `ωe·(Ld·id + λf)`; the PDF has the d-axis sign wrong)
- **averaged current sensing**: the controller uses the average current over the last period (`measureCurrents`)
- **field weakening**: a PI on voltage headroom (AMK's voltage controller, Kp 0.08 A/V, Tn 6 ms) makes `id*`
  negative once the voltage command passes 95% of what the bus can make, down to −35 A rms
- **voltage limit**: max voltage = `V_bus / √3` (SVM). At the limit the d axis gets its voltage first
- **limits**: torque ≤ 21 Nm, total current ≤ 105 A rms, torque fades out over the last 2% before 20000 rpm
- aims the voltage half a period ahead (the rotor keeps turning while the voltage is held)
- `turnOff()` when the contactors open

### Motor — `Vehicle/Motor/MotorConfig.py` (Level 2: average-value d/q model)
```
did/dt = (vd − Rs·id + ωe·Lq·iq) / Ld
diq/dt = (vq − Rs·iq − ωe·(Ld·id + λf)) / Lq
Te     = 1.5 · (poles/2) · [λf·iq + (Ld − Lq)·id·iq]
Pe     = 1.5 · (vd·id + vq·iq)          P_copper = 1.5 · Rs · (id² + iq²)
```
- takes the inverter's 3 phase voltages and converts them itself (`update_from_phase_voltages`)
- speed is set by the car through the gearbox (the motor's own inertia update gets overridden)
- `turnOff()`: no current, but the rotor keeps turning with the car
- integrated with forward Euler at 5 µs

### Battery — `Vehicle/Battery/TractiveBatteryPack.py`, `Cell.py` (Level 2: Thevenin resistance)
- 140 cells in series × 3 in parallel, 5 Ah / 6.7 mΩ cells
- OCV from an SOC table (`Configs/sr17.py`), terminal voltage `V = OCV − I·R_pack`
- SOC by coulomb counting, `I²R` heat, temperature with no cooling (adiabatic)
- `save_history` (on by default) keeps every step in lists. Long runs turn it off.

### BMS — `Vehicle/BMS/BMSConfig.py`
- reads the pack every 50 ms (`updateFromPack`), holds its readings in between
- **latched shutdown** on cell over/undervoltage (4200 / 2800 mV) or over/undertemperature (60 / 0 °C):
  contactors stay open until reset, even if the cells recover
- **overpower is only flagged** (> 80 kW), not limited (see design decisions)

### Chassis — `VehicleConfig.updateChassis`
```
wheel force = 4 × torque × 11.81 / 0.2032 m   (capped at tire grip = 1.2 × weight, flagged when it is)
drag        = ½ · 1.225 · 1.2 m² · v²          rolling = 0.015 × weight
a           = (wheel force − drag − rolling) / (vehicle mass + battery mass)
```
No wheel slip, load transfer, gearbox loss or rotating inertia yet.

### Where the power goes (example, 10 900 rpm, 21 Nm, full pack)
`battery chemical → − battery I²R → pack terminals → − inverter 3% → motors → − copper → shafts`.
Every step is checked to balance (the motor tests check `motor power = shaft + copper` to within 1%).

---

## Parameters and where they come from

| Parameter | Value | Source |
|---|---|---|
| motors / gear ratio | 4 / 11.81 | team |
| motor max torque / speed / current | 21 Nm / 20000 rpm / 105 A rms (1.24 s) | AMK datasheet |
| motor Rs (per phase) | 0.0675 Ω | datasheet Rtt 0.135 Ω is terminal-to-terminal, ÷ 2 |
| motor λf | 0.0293 Wb | from ke = 18.8 V/1000 rpm (Kt would give 0.0245, 16% too low) |
| motor Lq / Ld | 0.24 / 0.12 mH | **swapped vs the datasheet labels**, only this way fits its curves |
| field weakening current | 35 A rms | datasheet "magn. current Im" |
| current / voltage controller gains | Kp 0.58/0.64, Tn 1.2 ms / Kp 0.08, Tn 6 ms | datasheet controller settings |
| VCU loop / BMS sample | 10 ms / 50 ms | team |
| BMS limits | 4200/2800 mV, 60/0 °C, 80 kW | `bms1.py` |
| battery | 140s3p, 5 Ah, 6.7 mΩ cells, 50.8 kg | `Battery/Configs/sr17.py` |
| tire radius / grip | 0.2032 m / μ 1.2 | `tire1.py` |
| vehicle mass | 204.1 kg + battery | `SR17.py` |
| **inverter efficiency** | 0.97 | **placeholder** |
| **switching frequency** | 10 kHz | **placeholder** |
| **drag area / rolling resistance** | 1.2 m² / 0.015 | **placeholders** |
| **driver mass** | not included | **placeholder** (`driver1` is 10 kg) |

---

## Testing

### Scenarios — the easy way to add tests
Each test is one row in `SCENARIOS` in `Tests/scenarioTest.py`:

```python
Scenario("half throttle from 60 km/h",
         pedal=hold(50), run_time_s=2.0, start_speed_kph=60,
         checks=[torqueAbout(10.5, after_s=0.05), accelerates(), noShutdown()]),
```

`Scenario` options: `pedal`, `checks`, `run_time_s`, `stop_at_distance_m`, `stop_when` (e.g.
`stopWhenBmsShutsDown`), `start_soc`, `start_temp_C`, `start_speed_kph`, `record_every_s` (log less often on
long runs), `vehicle`.

**Pedal shapes** (`pedals.py`): `hold(30)`, `ramp(to=100, over_s=0.8)`, `steps((0, 100), (2.0, 0))`,
`pulses(100, on_s, off_s)`.

**Checks** (`checks.py`): `torqueAbout`, `torqueNeverAbove`, `accelerates`, `coasts`, `staysParked`,
`reachesDistance`, `reachesSpeed`, `noOverpower`, `overpowerFlagged`, `noShutdown`, `shutsDown`.
`ALWAYS` runs on every scenario: torque never over the motor max (a short overshoot after a sudden step is
allowed), the VCU only changes on its loop and follows the pedal, the BMS flag matches what it measured.

```bash
pytest Tests/scenarioTest.py -v                         # every scenario, pass/fail
pytest Tests/scenarioTest.py -k lift                    # just the ones with "lift" in the name
python Tests/scenarioTest.py --plot                     # run all, plot each
python Tests/scenarioTest.py "lift off" --plot          # by name
python Tests/scenarioTest.py --save plots               # save PNGs instead
python Tests/scenarioTest.py --slow                     # also the long ones (SLOW_SCENARIOS)
```

A failure explains itself, e.g. `FAIL wrong on purpose: torque feedback reached 6.30 Nm, expected 10.0 Nm +/-0.25`.
Scenarios are plotted even when they fail.

Current scenarios: part throttle 30%, lift off at 2 s, 75 m accel event, low battery, hot pack, cold pack, and
(slow) full throttle until the battery runs out.

### Other tests
Run any of them with `python Tests/<name>.py`. Run `inverterTest` after touching the inverter, it takes a second
and catches broken math straight away.

---

## Results so far

| | Result |
|---|---|
| 75 m accel event | 3.98 s, 129 km/h (grip-limited, no driver mass) |
| 0–100 km/h | 2.82 s |
| top speed | 129 km/h (motors at the 20000 rpm limit) |
| peak pack power | ~151 kW (no power limiting in the sim, the BMS flags > 80 kW from ~1.6 s) |
| motor vs AMK datasheet | exact up to the knee, within −9% / +19% past it with field weakening |
| full throttle until empty | 11 min 30 s, 24.4 km (avg 127 km/h), 7.07 kWh, SOC 100% → 3.2%, pack 25 → 52.7 °C (no cooling), ends in undervoltage. Holds 129 km/h for ~520 s, then fades to 114 km/h as the pack voltage drops |

### How much to trust it
| Speed | Trust |
|---|---|
| 0 to ~95 km/h | good: the main errors are the placeholders (driver mass, drag) |
| ~95–129 km/h (field weakening) | reasonable: within ~20% of the AMK datasheet, slightly optimistic |

Good for: testing VCU logic and timing, comparing options (trends). Not yet for: absolute lap times or energy
per lap (placeholders, no validation against AMK-car data).

---

## Design decisions

- **The 80 kW rule is only flagged, never enforced.** The flashed VCU has its own power limiting algorithm.
  A limiter in the sim would duplicate or misrepresent it.
- **One motor + inverter simulated, × 4.** All four get the same request, so they'd be identical. This has to
  change when motors get different requests (traction control, torque vectoring).
- **Torque names match the log:** request (VCU, *MCM Torque Command*), command (inverter after its limits,
  *MCM Commanded Torque*), feedback (what the motor made, *MCM Torque Feedback*).
- **Averaged current sensing.** Sampling the current once per period let the average current drift off its
  target at high speed (the rotor turns ~60° per 100 µs at 20000 rpm), which caused a torque shortfall and
  uncontrolled field weakening. Averaging over the period fixes that.
- **Average inverter model, not switching.** Switching needs 0.5 µs steps (10× slower) and `pwmTest` showed
  the averages match within 2.5 V.
- **The Crows log is the old EMRAX car** (Cascadia inverter, ~340 V), so it can't validate the AMK numbers.
  Its pedal and speed traces can still be used as inputs.

---

## Known gaps and next steps

**Placeholders to fill (quick, change every result):** driver mass, drag area, rolling resistance,
inverter efficiency and switching frequency, EMRAX electrical values.

**Not modeled yet:**
- brakes and regen (`BPS` does nothing yet)
- wheel slip and load transfer (launch power while the wheels would spin isn't right)
- 4 separate motors / inverters
- battery current limit check, cooling, RC voltage recovery
- motor / inverter temperatures (the 105 A max is only allowed for 1.24 s, not enforced)
- inverter conduction and switching losses (flat 3% now)
- replaying pedals or speed from a log (`fromLog` TODO in `pedals.py`), lap mode

**Model notes:**
- field weakening is up to ~19% more torque than AMK's curves mid-range (theirs level off at constant power)
- the Ld/Lq swap is a judgment call (AMK's own Tn 1.2 ms hints the other way)
- a 0 → 100% pedal step overshoots ~11% for ~2.5 ms (realistic pedal ramps don't)

**Validation:** compare against AMK-car logs once it runs (same pedal input, compare speed, pack current, voltage).

---

## Connecting the real VCU firmware (software-in-the-loop)

Plan: compile the VCU C code for the PC and run it **in lockstep** with the sim.

```
sim: run 100 updates (10 ms of sim time) ─► send inputs (APPS, BPS, speeds, BMS, inverter feedback, time)
VCU: run ONE loop of the real firmware   ─► return outputs (torque per motor, limits, enables)
sim: next 100 updates with those outputs ...
```

What the C code needs:
1. the main loop as a function (`vcu_init()`, `vcu_step()`) instead of `while(1)` + delay
2. **time comes from the sim** (replace tick / timer reads in the SIL build), so the firmware sees exact 10 ms
3. a thin I/O layer: ADC / CAN reads and writes go through a struct the sim fills and reads

Either a DLL called with `ctypes` or an executable talking over stdin/stdout or a socket works.
On the sim side `VCUConfig.update()` becomes the adapter, and the car will need 4 separate motors.
No real-time needed: lockstep runs at whatever speed the computer manages.

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
| SVM | space vector modulation, gets V_bus/√3 out of the inverter (15% more than plain sine PWM) |
| duty cycle | fraction of each switching period a leg's upper IGBT is on |
| anti-windup | stop a PI's integral growing while its output is saturated |
| SIL / HIL | software- / hardware-in-the-loop |
