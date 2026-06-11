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

struct IMUData {
    int64_t timestamp_ns = 0;
    std::string foot;

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
    std::string phase = "MSt";
    float gyro_z_value = 0.0f;
    uint32_t detection_count = 0;
    bool valid = false;
};

struct ElmoCommand {
    int64_t timestamp_ns = 0;
    std::string foot;
    int32_t target_velocity = 0;
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

    std::string error_message;
    bool valid = false;
};

// Live motor feedback read back from an ELMO drive (CiA-402 objects):
//   position  = 0x6064 Position actual value   (counts, INT32)
//   velocity  = 0x606C Velocity actual value   (counts/sec, INT32)
//   current   = 0x6078 Current actual value    (per-mille of rated current, INT16)
// Each field carries its own valid flag because the values arrive as separate
// SDO upload responses; a field stays invalid until its first response lands.
struct ElmoMotorInfo {
    int64_t timestamp_ns = 0;
    std::string foot;

    int32_t position = 0;
    int32_t velocity = 0;
    int16_t current = 0;

    bool position_valid = false;
    bool velocity_valid = false;
    bool current_valid = false;
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
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_TYPES_HPP
