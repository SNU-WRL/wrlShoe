#include "motorized_shoe/gait_phase_detection_node.hpp"

#include <cmath>

namespace motorized_shoe {

GaitPhaseDetectionNode::GaitPhaseDetectionNode(const Config& cfg, DataBus& bus)
    : bus_(bus),
      left_fsm_(std::make_unique<GaitEventFSM>(cfg.gait_sampling_frequency, "Left")),
      right_fsm_(std::make_unique<GaitEventFSM>(cfg.gait_sampling_frequency, "Right")) {
    left_fsm_->set_thresholds(
        cfg.gait_thresholds.hs_threshold,
        cfg.gait_thresholds.ts_threshold,
        cfg.gait_thresholds.ho_threshold,
        cfg.gait_thresholds.to_threshold,
        cfg.gait_thresholds.swing_threshold,
        cfg.gait_thresholds.midstance_threshold,
        cfg.gait_thresholds.impact_threshold,
        cfg.gait_thresholds.min_swing_dwell_ms);

    right_fsm_->set_thresholds(
        cfg.gait_thresholds.hs_threshold,
        cfg.gait_thresholds.ts_threshold,
        cfg.gait_thresholds.ho_threshold,
        cfg.gait_thresholds.to_threshold,
        cfg.gait_thresholds.swing_threshold,
        cfg.gait_thresholds.midstance_threshold,
        cfg.gait_thresholds.impact_threshold,
        cfg.gait_thresholds.min_swing_dwell_ms);
}

void GaitPhaseDetectionNode::tick() {
    const SystemSnapshot s = bus_.snapshot();

    if (s.imu_left.valid && s.imu_left.msg_count != last_left_msg_count_) {
        process(s.imu_left, *left_fsm_, "Left");
        last_left_msg_count_ = s.imu_left.msg_count;
    }

    if (s.imu_right.valid && s.imu_right.msg_count != last_right_msg_count_) {
        process(s.imu_right, *right_fsm_, "Right");
        last_right_msg_count_ = s.imu_right.msg_count;
    }
}

void GaitPhaseDetectionNode::process(const IMUData& imu, GaitEventFSM& fsm, const char* foot) {
    const float accel_norm = std::sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);

    const float foot_angle = std::atan2(
        2.0f * (imu.rv_r * imu.rv_j + imu.rv_i * imu.rv_k),
        1.0f - 2.0f * (imu.rv_j * imu.rv_j + imu.rv_k * imu.rv_k));

    const auto event = fsm.check_state_transition(imu.gz, accel_norm, foot_angle);

    GaitPhase gait;
    // Anchor to the IMU sample's own timestamp rather than processing time —
    // downstream consumers (slip_perturbation_node) use this as the t0 for
    // their delay timer, so any extra latency between IMU arrival and gait
    // processing shows up as slip-onset error.
    gait.timestamp_ns = imu.timestamp_ns;
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
