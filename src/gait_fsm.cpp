#include "motorized_shoe/gait_fsm.hpp"

#include <algorithm>

namespace motorized_shoe {

const char* gait_state_to_string(GaitState state) {
    switch (state) {
        case GaitState::MidStance:
            return "MSt";
        case GaitState::HeelOff:
            return "HO";
        case GaitState::ToeOff:
            return "TO";
        case GaitState::Swing:
            return "Swing";
        case GaitState::HeelStrike:
            return "HS";
        case GaitState::ToeStrike:
            return "TSt";
        default:
            return "Unknown";
    }
}

GaitEventFSM::GaitEventFSM(float fs, const std::string& foot) : fs_(fs), foot_(foot) {
    fsm_state_.current_state = GaitState::MidStance;
}

void GaitEventFSM::set_thresholds(float hs, float ts, float ho, float to, float swing, float midstance,
                                  float impact, int min_swing_dwell_ms) {
    hs_threshold_ = hs;
    ts_threshold_ = ts;
    ho_threshold_ = ho;
    to_threshold_ = to;
    swing_threshold_ = swing;
    midstance_threshold_ = midstance;
    impact_threshold_ = impact;
    const float ms_per_sample = 1000.0f / fs_;
    const int n = static_cast<int>(static_cast<float>(min_swing_dwell_ms) / ms_per_sample + 0.5f);
    min_swing_samples_ = (n > 0) ? n : 1;
}

GaitEventFSM::GaitEvent GaitEventFSM::check_state_transition(float gyro_z, float accel_norm, float foot_angle) {
    // NOTE: gyro_z must be in rad/s, accel_norm in m/s^2, thresholds in same units
    // Thresholds are converted from deg/s to rad/s: -80°/s = -1.3963, -30°/s = -0.5236, -200°/s = -3.4907 rad/s
    (void)foot_angle;

    GaitEvent event;
    event.state = fsm_state_.current_state;
    event.gyro_z_value = gyro_z;
    event.detection_count = fsm_state_.detection_count;

    if (foot_ == "Left") {
        gyro_z = -gyro_z;
    }

    gyro_buffer_.push_back(gyro_z);
    accel_buffer_.push_back(accel_norm);

    if (gyro_buffer_.size() > BUFFER_SIZE) {
        gyro_buffer_.pop_front();
    }
    if (accel_buffer_.size() > BUFFER_SIZE) {
        accel_buffer_.pop_front();
    }

    const int min_midstance_len = static_cast<int>(0.2f * fs_);

    switch (fsm_state_.current_state) {
        case GaitState::MidStance: {
            const int window = std::min(static_cast<int>(accel_buffer_.size()), min_midstance_len);
            bool is_mst = true;
            for (int i = static_cast<int>(accel_buffer_.size()) - window;
                 i < static_cast<int>(accel_buffer_.size()); ++i) {
                if (accel_buffer_[i] > midstance_threshold_) {
                    is_mst = false;
                    break;
                }
            }
            if (is_mst) {
                fsm_state_.searching_start_idx = static_cast<int>(fsm_state_.detection_count);
            }
            if (gyro_z <= ho_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::HeelOff;
                ++fsm_state_.detection_count;
            }
            break;
        }
        case GaitState::HeelOff:
            if (gyro_z <= to_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::ToeOff;
                ++fsm_state_.detection_count;
            }
            break;
        case GaitState::ToeOff:
            if (accel_norm > swing_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::Swing;
                ++fsm_state_.detection_count;
                // Reset per-swing gating. The entry sample is itself an
                // impact-magnitude event (swing_threshold > impact_threshold),
                // so seed saw_impact_peak_ as true to handle gait cycles whose
                // only suprathreshold sample is the toe-off thrust.
                saw_impact_peak_ = (accel_norm > impact_threshold_);
                swing_samples_ = 0;
            }
            break;
        case GaitState::Swing:
            ++swing_samples_;
            if (accel_norm > impact_threshold_) {
                saw_impact_peak_ = true;
            }
            if (saw_impact_peak_ && swing_samples_ >= min_swing_samples_
                && accel_norm <= midstance_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::HeelStrike;
                ++fsm_state_.detection_count;
            }
            break;
        case GaitState::HeelStrike:
            if (gyro_z <= ts_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::ToeStrike;
                ++fsm_state_.detection_count;
            }
            break;
        case GaitState::ToeStrike: {
            const int window = std::min(static_cast<int>(accel_buffer_.size()), min_midstance_len);
            bool is_mst = true;
            for (int i = static_cast<int>(accel_buffer_.size()) - window;
                 i < static_cast<int>(accel_buffer_.size()); ++i) {
                if (accel_buffer_[i] > midstance_threshold_) {
                    is_mst = false;
                    break;
                }
            }
            if (is_mst) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::MidStance;
                ++fsm_state_.detection_count;
            }
            break;
        }
    }

    event.state = fsm_state_.current_state;
    event.detection_count = fsm_state_.detection_count;
    return event;
}

}  // namespace motorized_shoe
