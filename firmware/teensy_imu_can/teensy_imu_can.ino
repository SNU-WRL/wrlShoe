// Motorized-shoe IMU node: one Teensy (FlexCAN_T4) reading two BNO085s over
// UART (SH-2 mode) and streaming them on the Pi's can1 at 500 kbit/s.
//
// Frame format is UNCHANGED from the previous sketch (same IDs, same int16
// scaling), so the Pi app keeps working as is. What changed (2026-09-13):
//
//  1. Sensor resets are handled. A BNO085 that resets (supply dip, glitch on
//     its reset line, internal fault) comes back with every report disabled
//     and only tells the host through wasReset(). The old sketch never looked,
//     so that foot went silent for good -- the clean-cut dropouts seen on
//     2026-09-08 (one sensor) and 2026-09-11 (the other). Now the reports are
//     re-enabled as soon as the reset is seen.
//  2. Per-sensor timeout. No report for IMU_TIMEOUT_MS -> pulse that sensor's
//     reset line and re-enable; every 4th consecutive timeout re-opens the
//     UART link from scratch. A sensor that failed at boot is retried the
//     same way, so a late-powered foot still comes up.
//  3. Frames are sent as each report arrives instead of waiting for a full
//     accel+rv+gyro+mag set, so the gyro (the only channel the gait detector
//     uses) no longer depends on the other three. The magnetometer is not
//     requested at all (nothing consumes it). The Pi counts samples on the
//     gyro frame, so this is transparent to it.
//  4. A 1 Hz status frame per sensor (0x11F / 0x12F) carries gyro frames/s,
//     reset count, timeout count and the CAN tx-drop count, so the Pi log
//     shows WHY a sensor went quiet.
//
// Report rate is 100 Hz (10000 us) on purpose: the Pi's gait FSM converts all
// its time windows with sampling_frequency: 100. Do not raise it without
// changing that config.

#include <Adafruit_BNO08x.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;

// ---- CAN config -------------------------------------------------------
// CAN_LOOPBACK 1 = internal self-test. The controller ACKs its own frames, so
// txOk must climb with NO transceiver, NO wiring and NO Pi attached. If it
// stays frozen in this mode the fault is inside the Teensy config, not the bus.
// Set back to 0 before running against the real bus.
#define CAN_LOOPBACK 0
#define CAN_BAUD     500000

// ---- IMU config -------------------------------------------------------
#define BNO1_RESET 5
#define BNO2_RESET 6
#define REPORT_INTERVAL_US 10000   // 100 Hz, must match the Pi's sampling_frequency
#define IMU_TIMEOUT_MS     250     // no report for this long -> recover the sensor
#define IMU_REOPEN_EVERY   4       // every Nth consecutive timeout: full begin_UART
#define ENABLE_MAG         0       // magnetometer frames (0x113/0x123); unused on the Pi

// Optional hardware watchdog (Teensy 4.x only, needs Watchdog_t4.h from
// Teensyduino >= 1.54). Resets the board if loop() ever stalls for > 2 s.
#define USE_HW_WATCHDOG 0
#if USE_HW_WATCHDOG && defined(__IMXRT1062__)
#include <Watchdog_t4.h>
WDT_T4<WDT1> wdt;
#endif

// CAN ID mapping for IMU1 (Serial1) and IMU2 (Serial2)
#define CAN_ID_IMU1_RV     0x110
#define CAN_ID_IMU1_ACCEL  0x111
#define CAN_ID_IMU1_GYRO   0x112
#define CAN_ID_IMU1_MAG    0x113
#define CAN_ID_IMU1_STATUS 0x11F

#define CAN_ID_IMU2_RV     0x120
#define CAN_ID_IMU2_ACCEL  0x121
#define CAN_ID_IMU2_GYRO   0x122
#define CAN_ID_IMU2_MAG    0x123
#define CAN_ID_IMU2_STATUS 0x12F

// Extra UART receive buffer space for the two IMU ports (see setup()).
uint8_t serial1RxExtra[1024], serial2RxExtra[1024];

// TX health counters. txOk climbs when a frame is accepted into a mailbox;
// txDropped climbs when write() is refused, which means the TX mailboxes are
// backed up because nothing on the bus is ACKing us (or we are bus-off).
uint32_t txOk = 0, txDropped = 0;
uint32_t lastStatusMs = 0;
uint32_t lastTxOk = 0, lastTxDropped = 0, canResets = 0;

// Everything that belongs to one sensor.
struct Imu {
  Adafruit_BNO08x bno;
  HardwareSerial *port;
  int resetPin;
  const char *name;
  uint32_t idRv, idAccel, idGyro, idMag, idStatus;

  bool open = false;          // begin_UART() succeeded
  uint32_t lastEventMs = 0;   // last report of any kind
  uint32_t gyroFrames = 0;    // gyro frames sent this second
  uint16_t resets = 0;        // wasReset() seen (sensor rebooted)
  uint16_t timeouts = 0;      // recoveries forced by IMU_TIMEOUT_MS
  uint8_t consecutiveTimeouts = 0;
  sh2_SensorValue_t value;

  Imu(HardwareSerial *p, int rst, const char *n,
      uint32_t rv, uint32_t ac, uint32_t gy, uint32_t mg, uint32_t st)
      : bno(rst), port(p), resetPin(rst), name(n),
        idRv(rv), idAccel(ac), idGyro(gy), idMag(mg), idStatus(st) {}
};

Imu imu1(&Serial1, BNO1_RESET, "IMU1", CAN_ID_IMU1_RV, CAN_ID_IMU1_ACCEL,
         CAN_ID_IMU1_GYRO, CAN_ID_IMU1_MAG, CAN_ID_IMU1_STATUS);
Imu imu2(&Serial2, BNO2_RESET, "IMU2", CAN_ID_IMU2_RV, CAN_ID_IMU2_ACCEL,
         CAN_ID_IMU2_GYRO, CAN_ID_IMU2_MAG, CAN_ID_IMU2_STATUS);

// ---------------------------------------------------------------------------

void setup(void) {
  // Serial.begin(115200);
  // while (!Serial) delay(10);   // keep commented: would hang with no USB host

  // Own the reset lines during power settle.
  pinMode(BNO1_RESET, OUTPUT);
  pinMode(BNO2_RESET, OUTPUT);
  digitalWrite(BNO1_RESET, LOW);
  digitalWrite(BNO2_RESET, LOW);

  // Power-settle delay: lets the 5 V rail stabilize before touching the IMUs.
  delay(500);

  digitalWrite(BNO1_RESET, HIGH);
  digitalWrite(BNO2_RESET, HIGH);
  delay(300);

  // Enlarge the serial RX buffers. The default is 64 bytes, which at 3 Mbaud
  // fills in ~200 us. Both IMUs stream continuously while loop() is busy
  // packing and sending CAN frames, so give each port ~3.4 ms of headroom.
  // Must be called once, before begin_UART() opens the ports.
  Serial1.addMemoryForRead(serial1RxExtra, sizeof(serial1RxExtra));
  Serial2.addMemoryForRead(serial2RxExtra, sizeof(serial2RxExtra));

  // CAN first so the status frames can report a sensor that fails to come up.
  canInit();

  openImu(imu1, 5);
  openImu(imu2, 5);

#if USE_HW_WATCHDOG && defined(__IMXRT1062__)
  WDT_timings_t cfg;
  cfg.timeout = 2;   // seconds
  wdt.begin(cfg);
#endif

  delay(100);
}

void loop() {
  serviceImu(imu1);
  serviceImu(imu2);

  // 1 Hz health: status frame per sensor + CAN deadlock recovery.
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();

    sendStatus(imu1);
    sendStatus(imu2);
    imu1.gyroFrames = 0;
    imu2.gyroFrames = 0;

    // Deadlock recovery: frames were refused this second and not one got
    // through. FlexCAN leaves bus-off on its own, but the software TX queue
    // stays jammed with frames that will never send, so re-init is the only
    // way out. This also means the link self-heals once the wiring is fixed,
    // with no power cycle needed.
    if (txDropped > lastTxDropped && txOk == lastTxOk) {
      canResets++;
      canInit();
    }
    lastTxOk = txOk;
    lastTxDropped = txDropped;
  }

#if USE_HW_WATCHDOG && defined(__IMXRT1062__)
  wdt.feed();
#endif
}

// ---------------------------------------------------------------------------
// IMU handling

// Open the UART link with `attempts` reset pulses; never blocks forever.
bool openImu(Imu &imu, int attempts) {
  imu.open = false;
  for (int attempt = 0; attempt < attempts && !imu.open; attempt++) {
    if (imu.bno.begin_UART(imu.port)) {
      imu.open = true;
      break;
    }
    pulseReset(imu.resetPin);
  }
  if (imu.open) {
    delay(100);
    setReports(imu);
    // begin_UART() itself resets the sensor; consume that flag so it is not
    // counted as a spontaneous reset.
    (void)imu.bno.wasReset();
  }
  imu.lastEventMs = millis();
  return imu.open;
}

void pulseReset(int pin) {
  digitalWrite(pin, LOW);
  delay(50);
  digitalWrite(pin, HIGH);
  delay(300);
}

void setReports(Imu &imu) {
  imu.bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US);
  imu.bno.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US);
  imu.bno.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US);
#if ENABLE_MAG
  imu.bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_INTERVAL_US);
#endif
}

// Drain one event, send its frame, and run the reset / timeout recovery.
void serviceImu(Imu &imu) {
  if (imu.open && imu.bno.getSensorEvent(&imu.value)) {
    imu.lastEventMs = millis();
    imu.consecutiveTimeouts = 0;
    handleEvent(imu, &imu.value);
  }

  // A reset drops every enabled report: re-enable immediately.
  if (imu.open && imu.bno.wasReset()) {
    imu.resets++;
    delay(20);
    setReports(imu);
    imu.lastEventMs = millis();
  }

  // Silent sensor: pulse its reset line (the reset advertisement then
  // triggers the re-enable above). Every IMU_REOPEN_EVERY-th consecutive
  // timeout, or if the port never opened, redo the whole UART bring-up.
  if (millis() - imu.lastEventMs > IMU_TIMEOUT_MS) {
    imu.timeouts++;
    imu.consecutiveTimeouts++;
    if (!imu.open || (imu.consecutiveTimeouts % IMU_REOPEN_EVERY) == 0) {
      openImu(imu, 1);
    } else {
      imu.bno.hardwareReset();
    }
    imu.lastEventMs = millis();   // back off one full timeout before retrying
  }
}

void handleEvent(Imu &imu, sh2_SensorValue_t *v) {
  int16_t c[4];
  uint8_t bytes[8];
  switch (v->sensorId) {
    case SH2_GYROSCOPE_CALIBRATED:
      c[0] = (int16_t)(v->un.gyroscope.x * 1000);
      c[1] = (int16_t)(v->un.gyroscope.y * 1000);
      c[2] = (int16_t)(v->un.gyroscope.z * 1000);
      memcpy(bytes, c, 6);
      if (sendframe(imu.idGyro, bytes, 6)) imu.gyroFrames++;
      break;
    case SH2_ROTATION_VECTOR:
      c[0] = (int16_t)(v->un.rotationVector.real * 10000);
      c[1] = (int16_t)(v->un.rotationVector.i * 10000);
      c[2] = (int16_t)(v->un.rotationVector.j * 10000);
      c[3] = (int16_t)(v->un.rotationVector.k * 10000);
      memcpy(bytes, c, 8);
      sendframe(imu.idRv, bytes, 8);
      break;
    case SH2_ACCELEROMETER:
      // NOTE: int16 mm/s^2 clips at +-32.77 m/s^2; heel-strike impacts do hit
      // this. Kept for Pi compatibility (the Pi divides by 1000). If accel is
      // ever needed, scale by 100 here and by 100 on the Pi.
      c[0] = (int16_t)(v->un.accelerometer.x * 1000);
      c[1] = (int16_t)(v->un.accelerometer.y * 1000);
      c[2] = (int16_t)(v->un.accelerometer.z * 1000);
      memcpy(bytes, c, 6);
      sendframe(imu.idAccel, bytes, 6);
      break;
#if ENABLE_MAG
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      c[0] = (int16_t)(v->un.magneticField.x * 1000);
      c[1] = (int16_t)(v->un.magneticField.y * 1000);
      c[2] = (int16_t)(v->un.magneticField.z * 1000);
      memcpy(bytes, c, 6);
      sendframe(imu.idMag, bytes, 6);
      break;
#endif
    default:
      break;
  }
}

// Status frame, 8 bytes little-endian:
//   [0:2] gyro frames sent in the last second (expect 100)
//   [2:4] sensor reset count (wasReset)
//   [4:6] timeout-recovery count
//   [6:8] CAN tx-dropped count (whole node), low 16 bits
void sendStatus(Imu &imu) {
  uint16_t f[4] = {
    (uint16_t)imu.gyroFrames, imu.resets, imu.timeouts, (uint16_t)txDropped };
  uint8_t bytes[8];
  memcpy(bytes, f, 8);
  sendframe(imu.idStatus, bytes, 8);
}

// ---------------------------------------------------------------------------
// CAN

void canInit() {
  can0.begin();                // soft-resets the peripheral: clears bus-off, flushes mailboxes
  can0.setBaudRate(CAN_BAUD);
  can0.setMaxMB(16);
  can0.enableFIFO();
  can0.enableFIFOInterrupt();
  can0.onReceive(canSniff);
#if CAN_LOOPBACK
  can0.enableLoopBack(true);   // controller ACKs its own frames; no bus required
#endif
}

void canSniff(const CAN_message_t &msg) {
  (void)msg;
}

bool sendframe(uint32_t id, uint8_t *data, uint8_t length) {
  CAN_message_t msg;
  if (length > 8) length = 8;   // clamp BEFORE setting the DLC
  msg.id = id;
  msg.len = length;
  memcpy(msg.buf, data, length);
  if (can0.write(msg) <= 0) {
    txDropped++;
    return false;
  }
  txOk++;
  return true;
}
