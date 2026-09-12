# Standalone C++ Motorized Shoe App

A C++17 control application for a motorized shoe rig with IMU-driven gait
phase detection and ELMO drive command/status over SocketCAN. No ROS
dependencies.

Implemented nodes (one source file per node):
- `src/read_can_interpret_imu_node.cpp`
- `src/gait_phase_detection_node.cpp`
- `src/send_can_command_to_elmo_node.cpp`
- `src/read_can_malfunction_from_elmo_node.cpp`

Additional app entry point:
- `src/main_imu_gait.cpp` (IMU + gait only)

Runtime behavior:
- Main loop executes at `1000 Hz`.
- A full-system snapshot row is logged on every tick (1 kHz) and flushed to
  the CSV every 10 ticks.
- Config is loaded from the local YAML file in `config/motorized_shoe_params.yaml` by default.
- On exit (q / Ctrl-C) both drives receive target velocity 0 and the Shutdown
  controlword, so a run never leaves a wheel spinning or armed.

## Build

```bash
cd /home/yoonhoshin/wrlShoe/cpp_motorized_shoe
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
cd /home/yoonhoshin/wrlShoe/cpp_motorized_shoe
sudo ./build/motorized_shoe_app
```

Run IMU + gait only:

```bash
cd /home/yoonhoshin/wrlShoe/cpp_motorized_shoe
sudo ./build/motorized_shoe_imu_gait_app
```

Optional args:

```bash
sudo ./build/motorized_shoe_app \
  --config /home/yoonhoshin/wrlShoe/cpp_motorized_shoe/config/motorized_shoe_params.yaml \
  --log /tmp/motorized_shoe_log.csv
```

## CAN bring-up

Bring both interfaces up before running any of the apps. `can0` (ELMO drives)
runs at 1 Mbit/s, `can1` (IMU Teensy) at 500 kbit/s. The script also renames
the two MCP2515 channels by HAT port (spi0.0 -> can0, spi0.1 -> can1) because
the kernel probes them in the wrong order on this Pi.

```bash
./scripts/can_up.sh           # can0 @ 1 Mbit/s, can1 @ 500 kbit/s
./scripts/can_up.sh can0      # just one
CAN1_BITRATE=250000 ./scripts/can_up.sh
./scripts/can_down.sh         # tear down
```

Equivalent manual commands:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 500000
```

If can0 is brought up at the wrong bitrate the drives never ACK, the
controller drops to ERROR-PASSIVE and the app prints
`SYNC send failed ... errno=105: No buffer space available` on every tick.
Check with `ip -details -statistics link show can0`: state must be
ERROR-ACTIVE and RX/TX packets must count up.

## Notes

- CAN interfaces must be up before running (`can0` for ELMO, `can1` for IMU).
- Uses Linux SocketCAN (`PF_CAN`).
- Logging path defaults to `motorized_shoe_log.csv` in current working directory.
- ELMO faults: the drive's EMCY frame (`0x80+node`) is parsed as soon as it
  arrives and the CiA-402 error code (`0x603F`) is read on any fault seen via
  the statusword poll, so the console and the CSV (`status_*_error`) show WHY
  the drive tripped. Fault recovery (Fault Reset → Shutdown → target 0 →
  Switch On → Enable Operation, ~150 ms) runs on the command node's worker
  thread and never blocks the 1 kHz loop; it is retried every
  `elmo_config.fault_retry_ms` up to `elmo_config.fault_max_retries` times, then
  gives up with a message (press `s` then `r`, or restart). A slip cannot be
  armed while the slip foot's drive is faulted, disabled, or busy.
- IMU watchdog: a foot whose newest IMU frame is older than `imu_stale_ms`
  (default 100) is reported STALE on the console (and in the `imu_*_age_ms`
  CSV columns); the slip app refuses to arm, and disarms, when the slip foot's
  IMU is stale. An IMU that sends nothing at all is reported after 3 s.
- Forward slip (mode 2) timing: the stance estimator rejects implausible
  stances/cycles, predicts from the median stance fraction times the recent
  cadence, resets after pauses / FSM resyncs / faults / emergency stops, and
  cancels the slip if the toe-off arrives before the scheduled fire (see
  `slip_perturbation` keys in the YAML and docs/slip_log_review_2026-09-12.md).
- Standalone tools (`send_zero_velocity_to_elmo`, `disable_elmo_drive`) default
  to the YAML node ids: left = 126, right = 127.
- Motor feedback (PDO): each drive streams CiA-402 position (`0x6064`),
  velocity (`0x606C`), current (`0x6078`), and the drive-internal command
  (demand) values — velocity demand (`0x606B`) and current demand (`0x6074`) —
  via two SYNC-triggered TPDOs, mapped at init (`send_can_command_to_elmo_node`
  configures the TPDOs in NMT Pre-Operational before `NMT Start`):
    - TPDO1 (`0x180+id`): position + velocity (8 B)
    - TPDO2 (`0x280+id`): current + velocity demand + current demand (8 B)
  The demand values are what the ELMO's own control loops command the motor: the
  velocity setpoint after profile shaping, and the current the controller is
  requesting. (`current_demand` maps the CiA-402 torque-demand object `0x6074`;
  on a current-mode ELMO drive torque demand is the commanded current, in the
  same per-mille-of-rated scale as the `0x6078` current actual, so the two
  compare directly as command vs. measured.) These are distinct from the
  `0x60FF` target velocity we send and from the actual feedback above, and they
  ride in TPDO2's spare bytes, so they add no extra bus traffic.
  The TPDOs are transmission type 1 (on every SYNC). `read_can_malfunction_from_elmo_node`
  emits one SYNC (`0x80`) per control-loop tick (1 kHz), so feedback updates at
  1 kHz; it parses the TPDOs into `motor_{left,right}_{position,velocity,current,velocity_demand,current_demand}`.
  The CSV logger writes every tick, so the log captures this at the full 1 kHz.
  Statusword stays on a slow SDO poll (`elmo_config.status_poll_ms`, default
  500 ms) for fault detection; EMCY frames give immediate fault notification. Bus cost on can0: SYNC + 2 TPDOs × 2 drives at 1 kHz
  ≈ ~45% of a 1 Mbit/s bus; the IMU is on a separate bus (can1). No per-value
  SDO request frames. Drive temperature is not logged (no portable CiA-402
  object — needs the ELMO-specific index). PDO config is sent fresh on every
  init (volatile); it is not stored to drive flash.
