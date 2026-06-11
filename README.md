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
- Full-system snapshot logging is written every `10 ms` to CSV.
- Config is loaded from the local YAML file in `config/motorized_shoe_params.yaml` by default.

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

Bring both interfaces up at 1 Mbit/s before running any of the apps:

```bash
./scripts/can_up.sh           # both can0 and can1 at 1 Mbit/s
./scripts/can_up.sh can0      # just one
BITRATE=500000 ./scripts/can_up.sh
./scripts/can_down.sh         # tear down
```

Equivalent manual commands:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

## Notes

- CAN interfaces must be up before running (`can0` for ELMO, `can1` for IMU).
- Uses Linux SocketCAN (`PF_CAN`).
- Logging path defaults to `motorized_shoe_log.csv` in current working directory.
- ELMO fault recovery: if a drive reports a fault during operation, the command
  node sends velocity = 0 and re-runs the full init sequence (Fault Reset →
  Shutdown → Switch On → Enable Operation) to bring it back into Operation
  Enabled. The control loop blocks for ~0.5 s during this recovery.
- Motor feedback (PDO): each drive streams CiA-402 position (`0x6064`),
  velocity (`0x606C`), and current (`0x6078`) via two SYNC-triggered TPDOs,
  mapped at init (`send_can_command_to_elmo_node` configures the TPDOs in NMT
  Pre-Operational before `NMT Start`):
    - TPDO1 (`0x180+id`): position + velocity
    - TPDO2 (`0x280+id`): current
  The TPDOs are transmission type 1 (on every SYNC). `read_can_malfunction_from_elmo_node`
  emits one SYNC (`0x80`) per control-loop tick (1 kHz), so feedback updates at
  1 kHz; it parses the TPDOs into `motor_{left,right}_{position,velocity,current}`.
  The CSV logger still samples every 10 ms, so the log captures this at 100 Hz
  (raise the log rate to capture the full 1 kHz). Statusword stays on a 2 Hz SDO
  poll for fault detection. Bus cost on can0: SYNC + 2 TPDOs × 2 drives at 1 kHz
  ≈ ~45% of a 1 Mbit/s bus; the IMU is on a separate bus (can1). No per-value
  SDO request frames. Drive temperature is not logged (no portable CiA-402
  object — needs the ELMO-specific index). PDO config is sent fresh on every
  init (volatile); it is not stored to drive flash.
