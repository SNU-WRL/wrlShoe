#include "motorized_shoe/gait_fsm.hpp"

#include <algorithm>
#include <numeric>

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

void GaitEventFSM::set_thresholds(float hs, float ts, float ho, float to, float swing_gyro,
                                  float midstance, int min_swing_dwell_ms) {
    hs_threshold_ = hs;
    ts_threshold_ = ts;
    ho_threshold_ = ho;
    to_threshold_ = to;
    swing_gyro_threshold_ = swing_gyro;
    midstance_threshold_ = midstance;
    const float ms_per_sample = 1000.0f / fs_;
    const int n = static_cast<int>(static_cast<float>(min_swing_dwell_ms) / ms_per_sample + 0.5f);
    min_swing_samples_ = (n > 0) ? n : 1;
}

void GaitEventFSM::set_filter_window(int window) {
    ma_window_ = (window > 1) ? window : 1;
    // Symmetric moving-average group delay is (window-1)/2 samples. Expressed in
    // nanoseconds so it can be subtracted from a back-dated peak timestamp.
    const int delay_samples = (ma_window_ - 1) / 2;
    const double sample_period_ns = 1e9 / static_cast<double>(fs_);
    ma_group_delay_ns_ = static_cast<int64_t>(delay_samples * sample_period_ns);
}

bool GaitEventFSM::update_peak(PeakDetector& d, float value, int64_t ts, float threshold,
                               int64_t& out_peak_ts) {
    if (!d.armed) {
        if (value <= threshold) {
            // Crossed down past the threshold: open the bracket.
            d.armed = true;
            d.min_value = value;
            d.min_timestamp_ns = ts;
        }
        return false;
    }

    // Below the threshold: keep tracking the deepest (most negative) sample.
    if (value < d.min_value) {
        d.min_value = value;
        d.min_timestamp_ns = ts;
    }
    if (value > threshold) {
        // Crossed back up past the threshold: fire, back-dated to the min sample.
        out_peak_ts = d.min_timestamp_ns;
        d.armed = false;
        return true;
    }
    return false;
}

GaitEventFSM::GaitEvent GaitEventFSM::check_state_transition(float gyro_z, float accel_norm,
                                                            float foot_angle, int64_t timestamp_ns) {
    // gyro_z in rad/s, accel_norm in m/s^2 (gravity removed), thresholds in same units.
    // TODO(optional fidelity): the original additionally gates the into-swing and
    // Swing->HS transitions on foot_angle local minima/maxima. foot_angle is
    // computed by the caller and passed here but not yet used.
    (void)foot_angle;

    if (foot_ == "Left") {
        gyro_z = -gyro_z;
    }

    // Moving-average filter feeding the FSM. The buffers hold the last ma_window_
    // raw samples; the filtered value is their mean.
    gyro_ma_buffer_.push_back(gyro_z);
    accel_ma_buffer_.push_back(accel_norm);
    if (static_cast<int>(gyro_ma_buffer_.size()) > ma_window_) {
        gyro_ma_buffer_.pop_front();
    }
    if (static_cast<int>(accel_ma_buffer_.size()) > ma_window_) {
        accel_ma_buffer_.pop_front();
    }
    const float gyro_f =
        std::accumulate(gyro_ma_buffer_.begin(), gyro_ma_buffer_.end(), 0.0f) /
        static_cast<float>(gyro_ma_buffer_.size());
    const float accel_f =
        std::accumulate(accel_ma_buffer_.begin(), accel_ma_buffer_.end(), 0.0f) /
        static_cast<float>(accel_ma_buffer_.size());

    accel_buffer_.push_back(accel_f);
    if (accel_buffer_.size() > BUFFER_SIZE) {
        accel_buffer_.pop_front();
    }

    GaitEvent event;
    event.state = fsm_state_.current_state;
    event.gyro_z_value = gyro_z;
    event.detection_count = fsm_state_.detection_count;
    event.event_timestamp_ns = timestamp_ns;  // default: this sample's time

    const int min_midstance_len = static_cast<int>(0.2f * fs_);

    switch (fsm_state_.current_state) {
        case GaitState::MidStance:
            // Heel-off: gyro_z drops below the (shallow) HO threshold. Detected on
            // the crossing itself (no back-date), matching the original.
            if (gyro_f <= ho_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::HeelOff;
                ++fsm_state_.detection_count;
                to_detector_.reset();
            }
            break;

        case GaitState::HeelOff: {
            // Toe-off: negative gyro_z peak, back-dated to the trough.
            int64_t peak_ts = 0;
            if (update_peak(to_detector_, gyro_f, timestamp_ns, to_threshold_, peak_ts)) {
                event.event_detected = true;
                event.event_timestamp_ns = peak_ts - ma_group_delay_ns_;
                fsm_state_.current_state = GaitState::ToeOff;
                ++fsm_state_.detection_count;
            }
            break;
        }

        case GaitState::ToeOff:
            // Into swing: forward (positive) gyro_z crosses the swing threshold.
            if (gyro_f >= swing_gyro_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::Swing;
                ++fsm_state_.detection_count;
                swing_samples_ = 0;
                hs_detector_.reset();
            }
            break;

        case GaitState::Swing: {
            // Heel-strike: negative gyro_z peak, back-dated to the trough, gated by
            // a minimum swing dwell so it cannot latch onto an early-swing dip.
            ++swing_samples_;
            int64_t peak_ts = 0;
            if (update_peak(hs_detector_, gyro_f, timestamp_ns, hs_threshold_, peak_ts)) {
                if (swing_samples_ >= min_swing_samples_) {
                    event.event_detected = true;
                    event.event_timestamp_ns = peak_ts - ma_group_delay_ns_;
                    fsm_state_.current_state = GaitState::HeelStrike;
                    ++fsm_state_.detection_count;
                }
                // A too-early crossing disarms the detector; it re-arms on the
                // next down-cross so the real (post-dwell) heel strike still fires.
            }
            break;
        }

        case GaitState::HeelStrike:
            // Toe-strike: gyro_z recovers UP past the TS threshold (matches the
            // original gyro_z[-2] < ts < gyro_z[-1] up-cross).
            if (have_prev_gyro_ && prev_gyro_ < ts_threshold_ && gyro_f > ts_threshold_) {
                event.event_detected = true;
                fsm_state_.current_state = GaitState::ToeStrike;
                ++fsm_state_.detection_count;
            }
            break;

        case GaitState::ToeStrike: {
            // Back to midstance once the free-accel norm is quiet over the window.
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

    prev_gyro_ = gyro_f;
    have_prev_gyro_ = true;

    event.state = fsm_state_.current_state;
    event.detection_count = fsm_state_.detection_count;
    return event;
}

}  // namespace motorized_shoe
