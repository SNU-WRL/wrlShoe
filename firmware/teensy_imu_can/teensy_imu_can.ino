// Motorized-shoe IMU node: one Teensy (FlexCAN_T4) reading two BNO085s over
// UART (SH-2 mode) and streaming them on the Pi's can1 at 500 kbit/s.
//
// Frame IDs, the 1 Hz status frame and the gyro / accel / rotation-vector
// scaling are UNCHANGED, so the Pi app keeps working as is. The calibrated
// magnetometer (0x113 / 0x123, logged since 2026-10-02) is sent as uT x 16,
// the BNO085's native Q4 (+-2048 uT); the Pi divides by 16. The old uT x 1000
// clipped at +-32.8 uT, below Earth's field. Bus load with mag: ~800
// frames/s, ~16% of 500 kbit/s.
//
// What changed vs readUART_sendCANIMU_reset (2026-09-15), and why:
//
//  The 2026-09-15 logs show every left-IMU dropout as an identical 694 ms
//  outage, with the healthy right IMU losing 3 x 30 ms + 365 ms alongside it.
//  Cause: the old serviceImu() ran its recovery with blocking delay()s in the
//  single loop, and its retry period (IMU_TIMEOUT_MS 50 + 30 ms pulse = 81 ms)
//  was shorter than the BNO085's reboot (~140 ms), so reset pulses 1-3 kept
//  re-resetting a sensor that was still booting. Only the 4th attempt
//  (begin_UART, which waits for the reset advertisement) ever worked.
//
//  1. Recovery is a per-sensor, non-blocking state machine. A timeout drops
//     the reset line for RESET_PULSE_MS, raises it, then keeps servicing the
//     port until the reset advertisement arrives (wasReset()) and re-enables
//     the reports. The other sensor is serviced normally the whole time.
//  2. Detection and back-off are separate. IMU_TIMEOUT_MS (50) still decides
//     WHEN a sensor is silent; RESET_WAIT_MS (300) is how long one reset gets
//     to produce an advertisement before it counts as failed.
//  3. begin_UART() (blocking, ~350 ms) is the fallback only, after
//     REOPEN_AFTER_FAILS reset pulses in a row produced no advertisement.
//     Its delay(100) is gone: getProdIds() already proved the hub is talking.
//  4. Bigger buffers so a block on one port can never destroy the other's
//     data: 4 KB UART RX per port (~470 ms at 4 reports x 100 Hz) and a
//     64-entry CAN TX queue for the burst that follows any block.
//
// Report rate is 100 Hz (10000 us) on purpose: the Pi's gait FSM converts all
// its time windows with sampling_frequency: 100. Do not raise it without
// changing that config.

#include <Adafruit_BNO08x.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> can0;

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
#define IMU_TIMEOUT_MS     50      // no report for this long -> start a recovery
#define RESET_PULSE_MS     10      // RSTN held low
#define RESET_WAIT_MS      300     // one reset gets this long to advertise (boot seen ~140 ms)
#define REOPEN_AFTER_FAILS 2       // failed pulses in a row before a full begin_UART
#define REOPEN_RETRY_MS    2000    // a port that never opened is retried this often
#define ENABLE_MAG         1       // calibrated magnetometer frames (0x113/0x123, uT x 16), logged on the Pi

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
// ~8.7 KB/s per sensor at 4 reports x 100 Hz (gyro, rv, accel, mag), so 4 KB
// is ~470 ms of headroom: still longer than the worst remaining block (the
// begin_UART fallback, ~350 ms).
uint8_t serial1RxExtra[4096], serial2RxExtra[4096];

// TX health counters. txOk climbs when a frame is accepted into a mailbox;
// txDropped climbs when write() is refused, which means the TX mailboxes are
// backed up because nothing on the bus is ACKing us (or we are bus-off).
uint32_t txOk = 0, txDropped = 0;
uint32_t lastStatusMs = 0;
uint32_t lastTxOk = 0, lastTxDropped = 0, canResets = 0;

// Recovery state of one sensor (see serviceImu()).
enum RecoveryState : uint8_t {
  RS_RUNNING,     // reports flowing, or silent but not yet timed out
  RS_RESET_LOW,   // RSTN held low, waiting RESET_PULSE_MS
  RS_RESET_WAIT   // RSTN released, waiting for the reset advertisement
};

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
  uint16_t resets = 0;        // spontaneous resets (wasReset() with no recovery running)
  uint16_t timeouts = 0;      // recoveries started by IMU_TIMEOUT_MS
  RecoveryState rstate = RS_RUNNING;
  uint32_t rstateMs = 0;      // when rstate was entered
  uint8_t failedPulses = 0;   // reset pulses in a row that produced no advertisement
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

  // Enlarge the serial RX buffers (default 64 bytes fills in ~200 us at
  // 3 Mbaud). Must be called once, before begin_UART() opens the ports.
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

// Full UART bring-up with `attempts` reset pulses. BLOCKING (~350 ms per
// attempt: reset, soft reset, advertisement wait, product IDs, report
// enables). Used at boot and as the last-resort recovery only.
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
    // No settle delay: begin_UART() returned only after the hub answered
    // sh2_getProdIds(), so it is ready for the report enables now.
    setReports(imu);
    // begin_UART() itself resets the sensor; consume that flag so it is not
    // counted as a spontaneous reset.
    (void)imu.bno.wasReset();
  }
  imu.rstate = RS_RUNNING;
  imu.failedPulses = 0;
  imu.lastEventMs = millis();
  return imu.open;
}

// Boot-time only (blocking); the runtime recovery pulses the pin itself.
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

// Begin a recovery for a silent sensor. Non-blocking: drops RSTN and hands
// the rest to the state machine in serviceImu(). Falls back to the blocking
// openImu() only when the port never opened or the pulses keep failing.
void startRecovery(Imu &imu) {
  imu.timeouts++;
  if (!imu.open || imu.failedPulses >= REOPEN_AFTER_FAILS) {
    openImu(imu, 1);
    return;
  }
  digitalWrite(imu.resetPin, LOW);
  imu.rstate = RS_RESET_LOW;
  imu.rstateMs = millis();
}

// Drain reports, send frames, and step the recovery state machine.
// Nothing in here blocks except setReports() (a few ms per enable) and the
// last-resort openImu().
void serviceImu(Imu &imu) {
  const uint32_t now = millis();

  // 1. Reports. The library read is non-blocking, and this keeps running
  //    during a recovery so the reset advertisement gets parsed.
  if (imu.open && imu.bno.getSensorEvent(&imu.value)) {
    imu.lastEventMs = now;
    imu.failedPulses = 0;
    handleEvent(imu, &imu.value);
  }

  // 2. Reset advertisement (ours or spontaneous): the hub comes back with
  //    every report disabled, so re-enable them and resume.
  if (imu.open && imu.bno.wasReset()) {
    if (imu.rstate == RS_RUNNING) {
      imu.resets++;               // nobody asked for it: count as spontaneous
    }
    setReports(imu);
    imu.rstate = RS_RUNNING;
    imu.failedPulses = 0;
    imu.lastEventMs = millis();   // reports start within a few intervals
    return;
  }

  // 3. Recovery state machine.
  switch (imu.rstate) {
    case RS_RUNNING: {
      // A port that never opened (unplugged foot) can only be retried with
      // the blocking openImu(), so pace that at REOPEN_RETRY_MS instead of
      // starving the other sensor every IMU_TIMEOUT_MS.
      const uint32_t limit = imu.open ? IMU_TIMEOUT_MS : REOPEN_RETRY_MS;
      if (now - imu.lastEventMs > limit) {
        startRecovery(imu);
      }
      break;
    }

    case RS_RESET_LOW:
      if (now - imu.rstateMs >= RESET_PULSE_MS) {
        digitalWrite(imu.resetPin, HIGH);
        imu.rstate = RS_RESET_WAIT;
        imu.rstateMs = now;
      }
      break;

    case RS_RESET_WAIT:
      if (now - imu.rstateMs > RESET_WAIT_MS) {
        // No advertisement: the pulse did nothing (line not wired? hub hung
        // hard?). Try again right away; after REOPEN_AFTER_FAILS of these
        // startRecovery() escalates to the full bring-up.
        imu.failedPulses++;
        imu.rstate = RS_RUNNING;
        startRecovery(imu);
      }
      break;
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
      // uT x 16 = the BNO085's native Q4, so nothing is lost and the range
      // (+-2048 uT) covers the sensor's own. The Pi divides by 16.
      c[0] = (int16_t)lroundf(constrain(v->un.magneticField.x * 16, -32768, 32767));
      c[1] = (int16_t)lroundf(constrain(v->un.magneticField.y * 16, -32768, 32767));
      c[2] = (int16_t)lroundf(constrain(v->un.magneticField.z * 16, -32768, 32767));
      memcpy(bytes, c, 6);
      sendframe(imu.idMag, bytes, 6);
      break;
#endif
    default:
      break;
  }
}

// Status frame, 8 bytes little-endian (unchanged layout):
//   [0:2] gyro frames sent in the last second (expect 100)
//   [2:4] spontaneous sensor reset count (wasReset with no recovery running)
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
