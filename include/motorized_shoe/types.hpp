#ifndef MOTORIZED_SHOE_TYPES_HPP
#define MOTORIZED_SHOE_TYPES_HPP

#include <cstdint>
#include <string>
#include <chrono>

namespace motorized_shoe {

inline int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 1 Hz health frame from the Teensy IMU node (CAN id 0x11F / 0x12F, see
// firmware/teensy_imu_can): gyro frames sent in the last second, sensor reset
// count, timeout-recovery count, CAN tx-drop count. Older sketches never send
// it; `valid` stays false then.
struct ImuNodeStatus {
    int64_t timestamp_ns = 0;
    uint16_t gyro_hz = 0;
    uint16_t resets = 0;
    uint16_t timeouts = 0;
    uint16_t tx_dropped = 0;
    bool valid = false;
};

struct IMUData {
    // Time of the newest gyro frame (the sample that drives the gait FSM).
    // NOT touched by the status frame, so the staleness watchdog sees a dead
    // sensor even while the Teensy keeps reporting on it.
    int64_t timestamp_ns = 0;
    std::string foot;
    ImuNodeStatus node_status;

    float rv_r = 0.0f;
    float rv_i = 0.0f;
    float rv_j = 0.0f;
    float rv_k = 0.0f;

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    uint32_t msg_count = 0;
    bool valid = false;
};

struct GaitPhase {
    int64_t timestamp_ns = 0;
    std::string foot;
    std::string phase = "Stance";
    float gyro_z_value = 0.0f;
    uint32_t detection_count = 0;
    bool valid = false;
};

struct ElmoCommand {
    int64_t timestamp_ns = 0;
    std::string foot;
    int32_t target_velocity = 0;
    // 0 gait-mapped, 1 fault stop + recovery, 2 external injection (slip),
    // 4 drive disabled (Shutdown), 5 drive re-enabled, 6 park at 0 (slip
    // mode), 7 stall-guard trip (drive disabled by check_stall).
    uint8_t command_type = 0;
    bool valid = false;
};

struct ElmoStatus {
    int64_t timestamp_ns = 0;
    std::string foot;

    uint16_t status_word = 0;
    bool fault = false;
    bool motor_enabled = false;
    bool operation_enabled = false;
    bool ready_to_switch_on = false;
    bool switched_on = false;

    // CiA-402 error code (0x603F) / EMCY error code of the current or most
    // recent fault, 0 when none has been reported. Populated from the drive's
    // EMCY frame (immediate) and from an 0x603F SDO read issued when the
    // statusword fault bit is first seen (fallback if no EMCY arrived).
    uint16_t error_code = 0;

    std::string error_message;
    bool valid = false;
};

// Live motor feedback + drive-internal commands read back from an ELMO drive
// (CiA-402 objects):
//   position        = 0x6064 Position actual value (counts, INT32)
//   velocity        = 0x606C Velocity actual value (counts/sec, INT32)
//   current         = 0x6078 Current actual value  (per-mille of rated current, INT16)
//   velocity_demand = 0x606B Velocity demand value (counts/sec, INT32) -- the
//                     drive's internal velocity setpoint after profile shaping
//   current_demand  = 0x6074 Torque demand value   (per-mille of rated torque,
//                     INT16) -- the controller output. On a current-mode ELMO
//                     drive this is the commanded current (torque is produced by
//                     q-axis current); same per-mille scale as `current` above,
//                     so the two compare directly as command vs. measured.
// Each field carries its own valid flag because the values arrive in separate
// TPDOs; a field stays invalid until its first frame lands.
struct ElmoMotorInfo {
    int64_t timestamp_ns = 0;
    std::string foot;

    int32_t position = 0;
    int32_t velocity = 0;
    int16_t current = 0;
    int32_t velocity_demand = 0;
    int16_t current_demand = 0;

    bool position_valid = false;
    bool velocity_valid = false;
    bool current_valid = false;
    bool velocity_demand_valid = false;
    bool current_demand_valid = false;
    bool valid = false;
};

struct SystemSnapshot {
    int64_t timestamp_ns = 0;
    IMUData imu_left;
    IMUData imu_right;
    GaitPhase gait_left;
    GaitPhase gait_right;
    ElmoCommand cmd_left;
    ElmoCommand cmd_right;
    ElmoStatus status_left;
    ElmoStatus status_right;
    ElmoMotorInfo motor_left;
    ElmoMotorInfo motor_right;

    uint32_t imu_node_latency_us = 0;
    uint32_t status_node_latency_us = 0;
    uint32_t gait_node_latency_us = 0;
    uint32_t command_node_latency_us = 0;
    uint32_t loop_latency_us = 0;

    // Cost of the logging path on the control-loop thread (snapshot copy + queue
    // + the every-10th-tick hand-off to the writer thread), which falls
    // *outside* loop_latency_us. Since 2026-09-15 the file write itself runs
    // on the logger's writer thread, so SD-card stalls no longer show here;
    // they show as a growing log_queue_depth instead.
    // Recorded one tick late: the value in row N is the logging cost measured
    // during the previous tick (a snapshot can't carry its own logging time).
    uint32_t log_latency_us = 0;
    // Filled by DataLogger::queue_snapshot: rows queued but not yet written
    // when this row was queued, and cumulative rows dropped (queue full).
    uint32_t log_queue_depth = 0;
    uint32_t log_dropped = 0;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_TYPES_HPP
