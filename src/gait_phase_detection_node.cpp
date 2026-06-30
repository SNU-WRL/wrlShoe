#include "motorized_shoe/gait_phase_detection_node.hpp"

#include <cmath>

namespace motorized_shoe {

namespace {

// Rotate a sensor-frame vector into the global frame by the unit quaternion
// q = (w, x, y, z): v' = v + 2*w*(qv x v) + 2*(qv x (qv x v)), with qv = (x,y,z).
void rotate_by_quaternion(float w, float x, float y, float z, float vx, float vy, float vz,
                          float& ox, float& oy, float& oz) {
    // t = 2 * (qv x v)
    const float tx = 2.0f * (y * vz - z * vy);
    const float ty = 2.0f * (z * vx - x * vz);
    const float tz = 2.0f * (x * vy - y * vx);
    // v' = v + w*t + (qv x t)
    ox = vx + w * tx + (y * tz - z * ty);
    oy = vy + w * ty + (z * tx - x * tz);
    oz = vz + w * tz + (x * ty - y * tx);
}

}  // namespace

void GravityEstimator::update(float ax, float ay, float az) {
    if (calibrated) {
        return;
    }
    const float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm >= 9.0f && norm <= 11.0f) {
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        ++count;
        if (count >= samples_needed) {
            gx = static_cast<float>(sum_x / count);
            gy = static_cast<float>(sum_y / count);
            gz = static_cast<float>(sum_z / count);
            calibrated = true;
        }
    }
}

GaitPhaseDetectionNode::GaitPhaseDetectionNode(const Config& cfg, DataBus& bus)
    : bus_(bus),
      left_fsm_(std::make_unique<GaitEventFSM>(cfg.gait_sampling_frequency, "Left")),
      right_fsm_(std::make_unique<GaitEventFSM>(cfg.gait_sampling_frequency, "Right")) {
    left_fsm_->set_thresholds(
        cfg.gait_thresholds.hs_threshold,
        cfg.gait_thresholds.ts_threshold,
        cfg.gait_thresholds.ho_threshold,
        cfg.gait_thresholds.to_threshold,
        cfg.gait_thresholds.swing_gyro_threshold,
        cfg.gait_thresholds.midstance_threshold,
        cfg.gait_thresholds.min_swing_dwell_ms);
    left_fsm_->set_filter_window(cfg.gait_ma_window);

    right_fsm_->set_thresholds(
        cfg.gait_thresholds.hs_threshold,
        cfg.gait_thresholds.ts_threshold,
        cfg.gait_thresholds.ho_threshold,
        cfg.gait_thresholds.to_threshold,
        cfg.gait_thresholds.swing_gyro_threshold,
        cfg.gait_thresholds.midstance_threshold,
        cfg.gait_thresholds.min_swing_dwell_ms);
    right_fsm_->set_filter_window(cfg.gait_ma_window);

    left_gravity_.samples_needed = cfg.gravity_calib_samples;
    right_gravity_.samples_needed = cfg.gravity_calib_samples;
}

void GaitPhaseDetectionNode::tick() {
    const SystemSnapshot s = bus_.snapshot();

    if (s.imu_left.valid && s.imu_left.msg_count != last_left_msg_count_) {
        process(s.imu_left, *left_fsm_, left_gravity_, "Left");
        last_left_msg_count_ = s.imu_left.msg_count;
    }

    if (s.imu_right.valid && s.imu_right.msg_count != last_right_msg_count_) {
        process(s.imu_right, *right_fsm_, right_gravity_, "Right");
        last_right_msg_count_ = s.imu_right.msg_count;
    }
}

void GaitPhaseDetectionNode::process(const IMUData& imu, GaitEventFSM& fsm,
                                     GravityEstimator& gravity, const char* foot) {
    // Rotate the sensor-frame acceleration into the global frame, then subtract
    // the estimated gravity vector so the FSM sees free (gravity-removed) accel.
    float gax = 0.0f;
    float gay = 0.0f;
    float gaz = 0.0f;
    rotate_by_quaternion(imu.rv_r, imu.rv_i, imu.rv_j, imu.rv_k, imu.ax, imu.ay, imu.az,
                         gax, gay, gaz);
    gravity.update(gax, gay, gaz);
    const float fx = gax - gravity.gx;
    const float fy = gay - gravity.gy;
    const float fz = gaz - gravity.gz;
    const float accel_norm = std::sqrt(fx * fx + fy * fy + fz * fz);

    const float foot_angle = std::atan2(
        2.0f * (imu.rv_r * imu.rv_j + imu.rv_i * imu.rv_k),
        1.0f - 2.0f * (imu.rv_j * imu.rv_j + imu.rv_k * imu.rv_k));

    const auto event = fsm.check_state_transition(imu.gz, accel_norm, foot_angle, imu.timestamp_ns);

    GaitPhase gait;
    // For a detected event use its (possibly back-dated) timestamp so downstream
    // consumers (slip_perturbation_node) anchor their delay timer to the true
    // gyro-peak time. Between events, fall back to the IMU sample timestamp.
    gait.timestamp_ns = event.event_detected ? event.event_timestamp_ns : imu.timestamp_ns;
    gait.foot = foot;
    gait.phase = gait_state_to_string(event.state);
    gait.gyro_z_value = event.gyro_z_value;
    gait.detection_count = event.detection_count;
    gait.valid = true;

    bus_.update_gait(gait);

    if (event.event_detected) {
        // Record the transition itself (not every IMU sample) so downstream
        // consumers can recover the exact moment of phase change even if
        // their polling rate misses individual ticks.
        bus_.push_gait_event(gait);
    }
}

}  // namespace motorized_shoe
