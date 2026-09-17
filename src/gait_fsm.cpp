#include "motorized_shoe/gait_fsm.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace motorized_shoe {

const char* gait_state_to_string(GaitState state) {
    switch (state) {
        case GaitState::Stance:
            return "Stance";
        case GaitState::Swing:
            return "Swing";
        default:
            return "Unknown";
    }
}

GaitEventFSM::GaitEventFSM(float fs, const std::string& foot) : fs_(fs), foot_(foot) {}

void GaitEventFSM::set_thresholds(float hs, float to, int min_swing_dwell_ms) {
    hs_threshold_ = hs;
    to_threshold_ = to;
    const float ms_per_sample = 1000.0f / fs_;
    const int n = static_cast<int>(static_cast<float>(min_swing_dwell_ms) / ms_per_sample + 0.5f);
    min_swing_samples_ = (n > 0) ? n : 1;
}

void GaitEventFSM::set_state_timeout_ms(int ms) {
    const float ms_per_sample = 1000.0f / fs_;
    const int n = static_cast<int>(static_cast<float>(ms) / ms_per_sample + 0.5f);
    state_timeout_samples_ = (n > 0) ? n : 1;
}

void GaitEventFSM::set_hs_accel_veto(bool enabled, float impact_threshold) {
    hs_accel_veto_ = enabled;
    hs_impact_threshold_ = impact_threshold;
}

int GaitEventFSM::ms_to_samples(int ms, float fs) {
    const int n = static_cast<int>(static_cast<float>(ms) * fs / 1000.0f + 0.5f);
    return (n > 0) ? n : 1;
}

void GaitEventFSM::set_contact_hs(bool enabled, const ContactHsParams& params) {
    contact_hs_ = enabled;
    contact_ = params;
    swing_run_needed_ = ms_to_samples(params.swing_min_ms, fs_);
    jerk_holdoff_samples_ = ms_to_samples(params.jerk_holdoff_ms, fs_);
    flat_min_samples_ = ms_to_samples(params.flat_min_ms, fs_);
    settle_timeout_samples_ = ms_to_samples(params.settle_timeout_ms, fs_);
    flat_reset_samples_ = ms_to_samples(params.flat_reset_ms, fs_);
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

GaitEventFSM::GaitEvent GaitEventFSM::check_state_transition(float gyro_z, float accel_x,
                                                            float accel_y, float accel_z,
                                                            float foot_angle, int64_t timestamp_ns) {
    // gyro_z in rad/s, accel_* in m/s^2 (gravity NOT removed; the norm feeds the
    // optional HS impact veto, the sample-to-sample change feeds the contact-HS
    // contact trigger). Thresholds are in rad/s. foot_angle is computed by the
    // caller but currently unused.
    (void)foot_angle;
    const float raw_accel_norm =
        std::sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    float jerk = 0.0f;
    if (have_prev_accel_) {
        const float dx = accel_x - prev_accel_[0];
        const float dy = accel_y - prev_accel_[1];
        const float dz = accel_z - prev_accel_[2];
        jerk = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    prev_accel_[0] = accel_x;
    prev_accel_[1] = accel_y;
    prev_accel_[2] = accel_z;
    have_prev_accel_ = true;

    // Single-foot right side is on the bench; the Left-foot gyro sign flip is
    // preserved for symmetry.
    if (foot_ == "Left") {
        gyro_z = -gyro_z;
    }

    // Moving-average filter feeding the peak detectors.
    gyro_ma_buffer_.push_back(gyro_z);
    if (static_cast<int>(gyro_ma_buffer_.size()) > ma_window_) {
        gyro_ma_buffer_.pop_front();
    }
    const float gyro_f =
        std::accumulate(gyro_ma_buffer_.begin(), gyro_ma_buffer_.end(), 0.0f) /
        static_cast<float>(gyro_ma_buffer_.size());

    ++state_dwell_samples_;

    GaitEvent event;
    event.state = current_state_;
    event.gyro_z_value = gyro_z;
    event.detection_count = detection_count_;
    event.event_timestamp_ns = timestamp_ns;  // default: this sample's time

    // Resync guard: if a state dwells far longer than a normal cycle without
    // firing its event, a gyro peak was missed. Reset the detectors and cycle
    // clocks and drop to Stance so the machine cannot stall (we must not trade
    // the old accel jam for a gyro jam). No event is emitted.
    int timeout_samples = state_timeout_samples_;
    if (recent_cycle_ns_ > 0) {
        const int cyc = static_cast<int>(1.5 * static_cast<double>(recent_cycle_ns_) *
                                         static_cast<double>(fs_) / 1e9);
        if (cyc > timeout_samples) {
            timeout_samples = cyc;
        }
    }
    if (state_dwell_samples_ > timeout_samples) {
        to_detector_.reset();
        hs_detector_.reset();
        swing_samples_ = 0;
        state_dwell_samples_ = 0;
        recent_cycle_ns_ = 0;
        have_last_hs_ = false;
        impact_seen_in_swing_ = false;
        swing_gyro_run_ = 0;
        swing_evidence_ = false;
        stance_settled_ = true;
        flat_run_ = 0;
        current_state_ = GaitState::Stance;
        // Published as an explicit "RESET" event (counted like any other
        // transition) so downstream consumers -- the slip node's stance
        // estimator, the gait-velocity map -- can drop stale cycle state
        // instead of silently continuing from a jammed machine.
        ++detection_count_;
        event.event_detected = true;
        event.event_label = "RESET";
        event.state = current_state_;
        event.detection_count = detection_count_;
        return event;
    }

    if (contact_hs_) {
        return step_contact_hs(event, gyro_z, gyro_f, jerk, timestamp_ns);
    }

    switch (current_state_) {
        case GaitState::Stance: {
            // Toe-off: negative gyro_z peak, back-dated to the trough. On fire,
            // hand the cycle to Swing and reset the HS detector + swing dwell.
            int64_t peak_ts = 0;
            if (update_peak(to_detector_, gyro_f, timestamp_ns, to_threshold_, peak_ts)) {
                event.event_detected = true;
                event.event_label = "TO";
                event.event_timestamp_ns = peak_ts - ma_group_delay_ns_;
                ++detection_count_;
                current_state_ = GaitState::Swing;
                swing_samples_ = 0;
                state_dwell_samples_ = 0;
                impact_seen_in_swing_ = false;
                hs_detector_.reset();
            }
            break;
        }

        case GaitState::Swing: {
            // Heel-strike: negative gyro_z peak, back-dated to the trough, gated by
            // a minimum swing dwell so it cannot latch onto an early-swing dip and
            // (optionally) by a raw-accel impact spike during the dwell.
            ++swing_samples_;
            if (hs_accel_veto_ && raw_accel_norm > hs_impact_threshold_) {
                impact_seen_in_swing_ = true;
            }
            int64_t peak_ts = 0;
            if (update_peak(hs_detector_, gyro_f, timestamp_ns, hs_threshold_, peak_ts)) {
                const bool dwell_ok = swing_samples_ >= min_swing_samples_;
                const bool veto_ok = !hs_accel_veto_ || impact_seen_in_swing_;
                if (dwell_ok && veto_ok) {
                    const int64_t t_hs = peak_ts - ma_group_delay_ns_;
                    event.event_detected = true;
                    event.event_label = "HS";
                    event.event_timestamp_ns = t_hs;
                    ++detection_count_;
                    // HS->HS period feeds the resync timeout (and the slip node's
                    // warm-up fallback via the event stream).
                    if (have_last_hs_) {
                        recent_cycle_ns_ = t_hs - last_hs_ts_ns_;
                    }
                    last_hs_ts_ns_ = t_hs;
                    have_last_hs_ = true;
                    current_state_ = GaitState::Stance;
                    state_dwell_samples_ = 0;
                    to_detector_.reset();
                }
                // A too-early or vetoed crossing disarms the detector; it re-arms
                // on the next down-cross so the real heel strike still fires.
            }
            break;
        }
    }

    event.state = current_state_;
    event.detection_count = detection_count_;
    return event;
}

void GaitEventFSM::enter_swing_contact() {
    current_state_ = GaitState::Swing;
    swing_samples_ = 0;
    state_dwell_samples_ = 0;
    swing_gyro_run_ = 0;
    swing_evidence_ = false;
    flat_run_ = 0;
    hs_detector_.reset();
}

GaitEventFSM::GaitEvent GaitEventFSM::step_contact_hs(GaitEvent event, float gyro_raw, float gyro_f,
                                                   float jerk, int64_t timestamp_ns) {
    flat_run_ = (std::fabs(gyro_f) < contact_.flat_gyro_max) ? flat_run_ + 1 : 0;
    swing_gyro_run_ = (gyro_raw > contact_.swing_gyro_min) ? swing_gyro_run_ + 1 : 0;

    switch (current_state_) {
        case GaitState::Stance: {
            if (!stance_settled_) {
                // Between the (contact) HS and foot-flat: the slap trough passes
                // here and must not reach the TO detector, and the residual
                // positive rotation right after contact is not a new swing.
                if (flat_run_ >= flat_min_samples_ ||
                    state_dwell_samples_ >= settle_timeout_samples_) {
                    stance_settled_ = true;
                    to_detector_.reset();
                }
                swing_gyro_run_ = 0;
                break;
            }
            int64_t peak_ts = 0;
            if (update_peak(to_detector_, gyro_f, timestamp_ns, to_threshold_, peak_ts)) {
                event.event_detected = true;
                event.event_label = "TO";
                event.event_timestamp_ns = peak_ts - ma_group_delay_ns_;
                ++detection_count_;
                enter_swing_contact();
            } else if (swing_gyro_run_ >= swing_run_needed_) {
                // Real swing rotation with no toe-off event: the push-off trough
                // was too weak (first step from standing). Follow the foot into
                // Swing so the coming landing is labelled HS, not TO.
                event.event_detected = true;
                event.event_label = "SWING";
                ++detection_count_;
                enter_swing_contact();
                swing_evidence_ = true;
                evidence_samples_ = 0;
                swing_peak_ = gyro_raw;
            }
            break;
        }

        case GaitState::Swing: {
            ++swing_samples_;
            bool hs = false;
            if (!swing_evidence_) {
                if (swing_gyro_run_ >= swing_run_needed_) {
                    swing_evidence_ = true;
                    evidence_samples_ = 0;
                    swing_peak_ = gyro_raw;
                }
            } else {
                ++evidence_samples_;
                swing_peak_ = std::max(swing_peak_, gyro_raw);
                const bool impact = jerk >= contact_.jerk_threshold &&
                                    gyro_raw < swing_peak_ - contact_.jerk_gyro_drop &&
                                    evidence_samples_ >= jerk_holdoff_samples_;
                hs = impact || gyro_raw < 0.0f;
            }
            if (hs) {
                // Stamped with this sample's time: both triggers are on raw
                // signals at the contact itself, nothing to back-date.
                event.event_detected = true;
                event.event_label = "HS";
                ++detection_count_;
                if (have_last_hs_) {
                    recent_cycle_ns_ = timestamp_ns - last_hs_ts_ns_;
                }
                last_hs_ts_ns_ = timestamp_ns;
                have_last_hs_ = true;
                current_state_ = GaitState::Stance;
                state_dwell_samples_ = 0;
                stance_settled_ = false;
                flat_run_ = 0;
                swing_gyro_run_ = 0;
                swing_evidence_ = false;
                to_detector_.reset();
            } else if (flat_run_ >= flat_reset_samples_) {
                // Foot flat and still while we say Swing: the landing never
                // produced a trigger (weak / shuffled step). The HS time is
                // unknown, so resync like the timeout guard does.
                event.event_detected = true;
                event.event_label = "RESET";
                ++detection_count_;
                current_state_ = GaitState::Stance;
                state_dwell_samples_ = 0;
                stance_settled_ = true;
                swing_gyro_run_ = 0;
                swing_evidence_ = false;
                recent_cycle_ns_ = 0;
                have_last_hs_ = false;
                to_detector_.reset();
                hs_detector_.reset();
            }
            break;
        }
    }

    event.state = current_state_;
    event.detection_count = detection_count_;
    return event;
}

}  // namespace motorized_shoe
