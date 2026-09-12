# Slip-log review and code changes — 2026-09-12

Context: the Pi's system image was lost after the 2026-09-11 session; the
repo was re-cloned at `e1257fb` and the only surviving uncommitted edits were
`config/motorized_shoe_params.yaml` (slip 300 ms, ramp 5e6) and the CAN
bring-up scripts. This note records what the 2026-09-08 … 09-11 slip logs
show, so the conclusions survive the next crash.

## What the logs show

Files: `2026091*_slip_*_log.csv` (1 kHz rows; `motor_*` columns are the TPDO
feedback, `status_*_word` is the 2 Hz statusword poll).

1. **Every Sep-10 slip tripped the drive mid-burst.** Timeline of a typical
   one (`20260910_144531`, ramp 1e7, right foot):

   | t (s)  | event |
   |--------|-------|
   | 22.211 | gyro trough (true heel strike) |
   | 22.301 | HS declared → `+150000` injected (90 ms after the trough) |
   | 22.304 | current already 569 ‰, 22.310: 1005 ‰, 22.316–22.370: pinned at **1767 ‰** (peak limit) |
   | 22.373 | drive drops the motor: velocity demand 0, current 0, wheel freewheels at 100–160 k counts/s |
   | 22.53  | statusword poll sees fault (0x0218) → blocking `stop_and_reset_elmo` |
   | 23.144 | loop resumes (613 ms gap in the log), drive re-armed |

   Same pattern in `144820` (ramp 5e6; motor off 93 ms after start) and
   `145132` (left-foot slip; the *right* drive tripped at 14.31 s while its
   wheel read 650 k counts/s). While loaded, the wheel could not follow the
   150 k demand: actual oscillated 10–130 k with the current pegged for
   60–90 ms, then the drive's own protection tripped. The Sep-11 slips
   (`124327`, three slips, no faults) reached 150 k within ~30 ms with the
   current only briefly above 1000 ‰, i.e. the foot was much less loaded.
   **The fault reason was never logged** (no EMCY parsing, no 0x603F read), so
   whether it is a velocity tracking error, over-speed or peak-current limit is
   still unknown. This is what the new EMCY / error-code logging is for.

2. **Faults also happen right after init** when the foot is moving during
   bring-up (`144455` right at 2.6 s, `144820` both feet at 1.6/2.7 s): the
   drive was enabled while the wheel was back-driven at 16–34 k counts/s and
   tripped within 0.6 s. In `144455` the recovery re-init did not clear it and,
   because recovery was attempted exactly once per episode, the drive stayed
   faulted for the whole 25 s run — the slip could never fire.

3. **Fault recovery blocked the 1 kHz loop** for 613 ms per drive (1225 ms
   with both), visible as `tick dt max` and as 616/1231 ms IMU gaps. During
   the block no SYNC was sent, the slip-end deadline was missed, and the
   re-init ran on the control thread while the init thread could still be
   using the same CAN socket.

4. **IMU dropouts**: right IMU died at 46.7 s on 2026-09-08 (clean cut, no
   noise precursor); left IMU ran at 37 Hz then died at 39.7 s on
   2026-09-11 and was absent in the later runs. Nothing in the app reported
   it; the gait FSM simply froze.

5. **HS detection latency** (declared HS minus gyro trough) is 46–110 ms,
   mean ~60 ms, and an AfterHS slip with delay 0 inherits it 1:1. Replaying
   the logs through the FSM with an "early fire at the turning point"
   variant gave only ~10 ms and, at small margins, false heel strikes, so it
   was not adopted. The MA window (5 samples = 20 ms group delay) and the
   width of the trough set the floor.

6. Miscellany: `can_up.sh` in the working tree had can0 at 500 kbit/s (the
   2026-09-08 `errno=105` storm); the standalone tools defaulted to left=127
   right=126, the opposite of the YAML; the README said logging was 10 ms
   (it is every tick).

## Changes made (working tree, uncommitted)

- `scripts/can_up.sh`: merged the SPI-port rename logic from the stray
  `Can_up.sh` (deleted), can0 back to 1 Mbit/s, can1 500 kbit/s, txqueuelen
  100, prints the controller state.
- `read_can_malfunction_from_elmo_node`: parses EMCY frames (immediate fault +
  error code), reads 0x603F when the statusword shows a fault, prints
  "FAULT"/"fault cleared" transitions, no longer flags a foot as faulted on a
  mere SDO abort (which used to trigger a full re-init), statusword poll
  period configurable, kernel CAN filters.
- `send_can_command_to_elmo_node`: one worker thread runs every blocking
  drive procedure (init, re-enable, fault recovery); `tick()` never blocks.
  Fault recovery is a ~150 ms confirmed sequence (reset → shutdown → target 0
  → switch on → enable → statusword readback) retried every
  `fault_retry_ms` up to `fault_max_retries`. Target velocity 0 is written
  before Enable Operation in every bring-up. `release_external_control`
  holds 0 in slip mode instead of the Swing velocity. `stop_all_drives()`
  parks and disables both drives at exit. Slip ramp override is a
  constructor argument. `is_drive_available()` for the slip node.
- `slip_perturbation_node`: refuses to arm (with the reason) when the slip
  foot's IMU is stale or its drive is unavailable; disarms if the IMU dies
  while armed.
- `ImuWatchdog` (`imu_watchdog.hpp`): console lines on stale/fresh
  transitions and for an IMU that never sends; used by both apps.
- CSV: four columns appended — `imu_left_age_ms, imu_right_age_ms,
  status_left_error, status_right_error`.
- Config keys: `imu_stale_ms`, `elmo_config.status_poll_ms`,
  `elmo_config.fault_retry_ms`, `elmo_config.fault_max_retries`.
- Tool defaults left=126 / right=127; README updated.

## Next steps on the rig

1. Bring can0 up with the new script and confirm `ERROR-ACTIVE`.
2. Run the slip app, cause one slip; read the `EMCY error 0x....` /
   `error code (0x603F)` line. Then decide the drive-side fix: relax the
   speed tracking error window (Elmo `ER[2]`), lower the slip velocity /
   ramp, or raise the peak-current window. Until then expect faults on a
   fully loaded foot.
3. Watch for `[imu] ... STALE` lines; both cables/Teensys have failed.

## Addendum 2026-09-13 — IMU dropouts: cause and fix

Topology (confirmed): one Teensy reads both BNO085s over UART (SH-2 mode) and
sends both feet on can1; CAN IDs are fixed per UART port in the sketch. The
two dropouts (right 2026-09-08, left 2026-09-11) happened to the same physical
sensor after its UART line was moved to the other Teensy port: the
accelerometer gain error of the *other* sensor flipped label at the same time,
which is only possible if the labels moved with the lines. So the shoe harness
and the Teensy ports are cleared; the failure is in that sensor (or its
board-level wiring). The Teensy never hung: the other foot streamed at exactly
100 Hz through both cuts.

Sketch root cause: the old sketch never called `wasReset()`. A BNO085 that
resets (supply dip, reset-line glitch, internal fault) comes back with all
reports disabled and stays silent until the host re-enables them; the sketch
never did, so that foot was dead until a power cycle. The frame set was also
gated on all four reports, so the gyro depended on accel, rotation vector and
magnetometer.

`firmware/teensy_imu_can/teensy_imu_can.ino` (not compiled here, no Arduino
toolchain on the Pi): re-enables reports on `wasReset()`, 250 ms per-sensor
timeout with reset pulse / UART reopen, frames sent per report (magnetometer
not requested), 1 Hz status frame `0x11F`/`0x12F` = u16 gyro frames/s, resets,
timeouts, CAN tx drops. Frame format otherwise unchanged. Pi side: sample
count now advances on the gyro frame (compatible with both sketches), status
frame parsed into `imu_*_node_*` CSV columns and console lines on every reset
/ timeout. Accel still packs as int16 mm/s² (clips at 32.7 m/s²) to keep the
Pi decoding unchanged; scale both sides by 100 if accel is ever needed.
