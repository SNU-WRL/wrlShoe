# Gait phase detection + cycle percentage — implementation note

Status: **design / not yet implemented.** Captures the plan to simplify the
gait FSM to a two-event (HS/TO) detector and to emit a continuous gait-cycle
percentage (0–100%). Written 2026-06-11.

## Goal

Replace the implicit "phase = current FSM state name" output with a continuous
**gait-cycle percentage**. Define the cycle by a single repeating event and
linearly interpolate position within it in real time. Optionally collapse the
6-state FSM (`MidStance → HeelOff → ToeOff → Swing → HeelStrike → ToeStrike`)
down to a 2-event detector and derive sub-phases from percentage windows.

## Convention to anchor to

- Gait cycle = **heel-strike to heel-strike of the same foot**. HS = 0%, next
  ipsilateral HS = 100%.
- Toe-off lands at **~60%** at normal walking speed (stance ≈ 0–60%, swing ≈
  60–100%).
- Anchor to **HS→HS**, not TO→TO: HS=0% matches every published gait curve, and
  HS is the most reliably detected single event on a foot IMU (sharp accel
  impact *and* a gyro spike).

## Detection — common robust method

Primary signal is the **foot sagittal-plane angular velocity** = `gyro_z`
(already sign-normalized per foot in `gait_fsm.cpp`; left foot negates). Over one
cycle the signal has a repeatable shape: a large **positive mid-swing peak**,
with a **negative peak near toe-off** before it and a **negative peak near
heel-strike** after it (Aminian / Salarian / Sabatini method).

Canonical (offline) rule — find events relative to the mid-swing peak:
- Mid-swing = max positive `gyro_z` in the cycle.
- Heel strike = first negative peak / negative-going zero crossing **after**
  mid-swing, confirmed by an **accelerometer impact spike** in `|a|`
  (`accel_norm`). The accel impact is what makes HS robust.
- Toe off = the prominent negative peak **before** mid-swing. Use gyro; accel
  alone is weak for TO.

Real-time equivalent (what we'll implement — threshold machine, no lookahead).
Three robustness additions, in priority order:
1. **Refractory / lockout** after each event (reject a new HS within ~250–300
   ms). Highest-value fix; kills most double-triggers.
2. **Mid-swing gate**: don't accept HS unless a positive mid-swing peak has
   occurred since the last event. Prevents spurious events while standing.
3. **Hysteresis** (separate enter/exit thresholds) + light low-pass / short
   median filter on `gyro_z` so threshold jitter doesn't fire twice.

## Percentage computation

Real-time, time-normalized against the previous cycle:

```
phase% = clamp( (t_now - t_HS) / T_prev * 100, 0, 100 )
```

`t_HS` = timestamp of the last heel-strike. `T_prev` = duration of the last
completed HS→HS cycle. Robustness details:
- Use a **rolling average of the last 2–3 cycle durations** for `T_prev`, not
  just the single last cycle.
- **Reject outlier cycles**: if a measured duration is outside ~[0.5×, 1.5×] of
  the running median, drop it (missed / double event) — don't feed it into
  `T_prev`.
- **Clamp at 100%** when the next HS is late (subject slowing).
- **Flag invalid / "not walking"** if no event within ~1.5–2 s.

This is a one-cycle-ahead predictor, so it lags on cadence changes — accepted
and unavoidable without future info. If the *distribution* must be accurate (not
just monotonic), anchor two events instead (HS=0%, TO=~measured%) and interpolate
linearly within stance and within swing separately.

## Concrete code changes

- `include/motorized_shoe/types.hpp` (`GaitPhase`, ~line 41): add
  `float phase_percent = 0.0f;` (and optionally a `bool walking` flag).
- `include/motorized_shoe/gait_fsm.hpp` / `src/gait_fsm.cpp`:
  - Add state: `t_HS` (last HS timestamp), a small ring buffer of recent cycle
    durations, a `saw_midswing` gate flag, and the refractory lockout timestamp.
  - Either collapse to a 2-event (HS/TO) detector + STANCE/SWING flag and derive
    HO/MSt/etc. as percentage windows, OR keep the 6 states but add the
    refractory + mid-swing gate (the refractory is worth adding regardless).
  - Compute `phase_percent` from `t_HS`, `T_prev`, and the current sample
    timestamp.
- `src/gait_phase_detection_node.cpp` (`process()`, ~line 55): populate
  `gait.phase_percent`. Keep using `imu.timestamp_ns` as t0 (already correct).
- `src/data_logger.cpp`: add a `gait_*_phase_percent` column to the CSV header
  and row so it's captured in logs.
- Downstream (`slip_perturbation_node`): once `phase_percent` exists, the slip
  trigger can fire "at X% of cycle" instead of "X ms after MSt entry" — a
  cleaner trigger than the event+delay scheme in commit
  `0e08ff0`. Optional follow-up, not required.

## Config

Keep the existing threshold keys in `config/motorized_shoe_params.yaml` so
nothing downstream breaks. New keys to add when implementing:
- `refractory_ms` (default ~280)
- `cycle_history_len` (default 3) for the rolling `T_prev` average
- `no_walk_timeout_ms` (default ~1800) for the not-walking flag

## Caveats

- HS/TO percentage works at the current 120 Hz IMU rate, but the sharp TO gyro
  peak is better resolved at 200 Hz+ if TO timing jitter shows up.
- Validate against existing logs (`*_imu_gait_log.csv`, `*_slip_log.csv`) before
  trusting the percentage live.
