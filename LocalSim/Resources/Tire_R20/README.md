# Tire_R20 — Pacejka Magic Formula fit for the Hoosier R20 compound

Built from real Calspan TTC Round 9 (project 2356) raw run data, to replace
the monotonically-saturating Dugoff model in `SR_17.py`/`SR_18.py` with one
that has a genuine slip-ratio/slip-angle **peak followed by a falloff**,
which is what motivated this in the first place (see the launch-control
`target_slip_ratio` discussion — the Dugoff model never actually loses grip
past peak slip, so tight real-world-style targets like 0.10–0.20 were
needlessly leaving force on the table there).

## Data used

- `RunGuide_Round9.pdf` and `2356 Summary Tables.xlsx` (Test Schedule + Tire
  ID Schedule sheets) map every run number to its tire, size, pressure,
  camber, load, and sweep type.
- Raw per-run time-series files: `B2356run<N>.dat` (tab-separated: ET, V, N,
  SA, IA, RL, RE, P, FX, FY, FZ, MX, MZ, NFX, NFY, RST, TSTI/C/O, AmbTmp, SR).
- **All four Hoosier R20/C2000 carcass sizes are pooled together**: 16x6.0-10,
  16x7.5-10 (the actual car's tire), 18.0x6.0-10, and 20.5x7.0-13 — same
  rubber compound, different carcass, per request to use every R20 run to
  better-constrain the fit for the 16x7.5-10 tire specifically.
- **Important finding**: Hoosier ran no drive/brake (slip ratio) tests at all
  on either 16" R20 size this round — only cornering (slip angle) sweeps
  exist for 16x6.0-10/16x7.5-10. So the longitudinal fit is entirely
  extrapolated from the 18x6.0-10 and 20.5x7.0-13 R20 runs; only the lateral
  fit has any real 16" data behind it.

## Pipeline

1. `parse_ttc.py` — reads one `.dat` file's metadata/header/units and returns
   a pandas DataFrame.
2. `build_dataset.py` — walks the R20 cornering runs (2,4-21,27-32) and
   drive/brake runs (51,52,54,55,69,70,72,73), filters each to its pure-slip
   subset (cornering runs never spin the wheel — N/RE/SR are non-physical
   there, SR is a fixed -1.0 sentinel not a real reading, so kappa≈0 by test
   construction and no SR filter is needed or possible; drive/brake runs are
   filtered to |SA| < 0.5°), and writes `lateral_dataset.csv` (917,805 points)
   and `longitudinal_dataset.csv` (179,969 points).
3. `fit_pacejka.py` — fits a simplified per-axis Pacejka pure-slip form via
   `scipy.optimize.curve_fit`:
   ```
   D(Fz) = c1*Fz + c2*Fz^2        (peak force vs load)
   K(Fz) = c3*Fz + c4*Fz^2        (slip stiffness at origin vs load)
   B(Fz) = K(Fz) / (C * D(Fz))
   F(slip, Fz) = D(Fz)*sin(C*atan(B(Fz)*slip - E*(B(Fz)*slip - atan(B(Fz)*slip))))
   ```
   Results: longitudinal R²=0.947 (RMSE 403.6 N, n=22,497 after subsampling),
   lateral R²=0.988 (RMSE 170.5 N, n=114,725). Saves `pacejka_r20_coeffs.npz`
   and `pacejka_r20_fit.png` (fit vs raw scatter at several loads).
4. `Pacejka_R20.py` — the actual tire model class, same interface style as
   `SR_17`/`SR_18` (`tire_forces(fz, kappa, alpha)`, `tire_slip_ratio`).

## Key results

- Peak slip ratio ≈ 0.17–0.18 (falls in the realistic 0.1–0.2 launch-control
  range), peak slip angle ≈ 11–12°.
- Effective peak friction coefficient is **higher** than the Dugoff model's
  flat `tire_mu=1.5` at most loads actually driven in this project's sims
  (~2.2–2.6 at light-to-moderate Fz, decreasing with load — real load
  sensitivity) — see `pacejka_vs_dugoff.png`. This suggests the existing
  Dugoff-based sims may have been simulating conservatively (lower peak grip
  than the real tire has), on top of never modeling the post-peak falloff.
- Past peak, force does fall off (unlike Dugoff), but the raw fitted curve is
  only valid within the data's real range (kappa ∈ [-0.30, 0.23], alpha ∈
  [-12.5°, 12.6°]) — extrapolated further, the classic Pacejka
  `sin(C*atan(...))` form crosses back through zero and goes *negative*
  around kappa≈0.57, which is physically wrong and a real risk given this
  project's sims have already produced transient wheelspin above 70% slip.
  `Pacejka_R20.tire_forces` clamps to that data-supported range and holds the
  boundary value flat beyond it.

## Scope / not done

- **No camber term** — the team's own prior TTC review already found no
  meaningful camber sensitivity for this tire (see `SR_17.py`'s
  `tires_camber_sensitive` comment), so camber sweep points were pooled in
  as ordinary data rather than fit as a separate term.
- **No independent combined-slip fit** — this TTC round has no combined
  slip-ratio + slip-angle sweep to fit against. `Pacejka_R20.tire_forces`
  falls back to an approximate friction-circle scaling (clip both pure-slip
  values down if their combined magnitude, each normalized by its own axis's
  peak force at that Fz, exceeds 1) when both kappa and alpha are nonzero —
  not itself validated against data. This isn't currently exercised anywhere
  in the project: the Accel sims hold alpha=0 throughout, and Skidpad.py
  uses `tire_mu` directly for its grip circle rather than calling
  `tire_forces`.
- **Not yet wired into SR_17.py/SR_18.py or any sim** — this is a standalone
  model in its own folder. Swapping it in for the Dugoff model would mean
  replacing `SR_17.tire_forces`/`SR_18.tire_forces` (or adding it as an
  alternative), then re-validating every sim that depends on tire behavior
  (both Accel sims, Skidpad, Endurance, Autocross) since peak grip, peak
  slip location, and post-peak falloff all differ from what those sims were
  tuned/interpreted against so far.
