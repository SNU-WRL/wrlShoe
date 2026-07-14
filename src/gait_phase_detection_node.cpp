#include "motorized_shoe/gait_phase_detection_node.hpp"

#include <cmath>

namespace motorized_shoe {

GaitPhaseDetectionNode::GaitPhaseDetectionNode(const Config& cfg, DataBus& bus)
    : bus_(bus),
      left_fsm_(std::make_unique<GaitEventFSM>(cfg.gait_sampling_frequency, "Left")),
      right_fsm_(std::make_unique<GaitEventFSM>(cfg.gait_sampling_frequency, "Right")) {
    for (GaitEventFSM* fsm : {left_fsm_.get(), right_fsm_.get()}) {
        fsm->set_thresholds(cfg.gait_thresholds.hs_threshold,
                            cfg.gait_thresholds.to_threshold,
                            cfg.gait_thresholds.min_swing_dwell_ms);
        fsm->set_state_timeout_ms(cfg.gait_state_timeout_ms);
        fsm->set_hs_accel_veto(cfg.gait_thresholds.hs_accel_veto,
                               cfg.gait_thresholds.hs_impact_threshold);
        fsm->set_filter_window(cfg.gait_ma_window);
    }
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
    // Raw acceleration norm (gravity NOT removed). Only consumed by the optional
    // HS impact veto, where the impact spike (~30-40 m/s^2) dwarfs gravity.
    const float raw_accel_norm =
        std::sqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);

    const float foot_angle = std::atan2(
        2.0f * (imu.rv_r * imu.rv_j + imu.rv_i * imu.rv_k),
        1.0f - 2.0f * (imu.rv_j * imu.rv_j + imu.rv_k * imu.rv_k));

    const auto event =
        fsm.check_state_transition(imu.gz, raw_accel_norm, foot_angle, imu.timestamp_ns);

    GaitPhase gait;
    // For a detected event use its back-dated timestamp so downstream consumers
    // anchor to the true gyro-peak time; between events use the IMU sample time.
    gait.timestamp_ns = event.event_detected ? event.event_timestamp_ns : imu.timestamp_ns;
    gait.foot = foot;
    // Continuous phase is the FSM state (Stance/Swing): drives the gait-phase
    // velocity map and the CSV column.
    gait.phase = gait_state_to_string(event.state);
    gait.gyro_z_value = event.gyro_z_value;
    gait.detection_count = event.detection_count;
    gait.valid = true;

    bus_.update_gait(gait);

    if (event.event_detected) {
        // Push the transition under its explicit event label ("HS"/"TO") rather
        // than the state name, so the slip node consumes the markers directly.
        GaitPhase ev = gait;
        ev.phase = event.event_label;
        bus_.push_gait_event(ev);
    }
}

}  // namespace motorized_shoe
