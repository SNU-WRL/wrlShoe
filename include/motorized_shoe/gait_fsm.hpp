#ifndef MOTORIZED_SHOE_GAIT_FSM_HPP
#define MOTORIZED_SHOE_GAIT_FSM_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace motorized_shoe {

enum class GaitState {
    MidStance,
    HeelOff,
    ToeOff,
    Swing,
    HeelStrike,
    ToeStrike
};

const char* gait_state_to_string(GaitState state);

struct GaitFSMState {
    GaitState current_state = GaitState::MidStance;
    int searching_start_idx = 0;
    uint32_t detection_count = 0;
};

class GaitEventFSM {
public:
    GaitEventFSM(float fs = 120.0f, const std::string& foot = "Right");

    void set_thresholds(float hs, float ts, float ho, float to, float swing_gyro,
                        float midstance, int min_swing_dwell_ms);

    // Moving-average window (samples) on gyro_z and accel_norm. window <= 1
    // disables filtering. The (window-1)/2-sample group delay is compensated
    // for when back-dating event timestamps.
    void set_filter_window(int window);

    struct GaitEvent {
        GaitState state = GaitState::MidStance;
        float gyro_z_value = 0.0f;
        uint32_t detection_count = 0;
        bool event_detected = false;
        // Timestamp the event is attributed to. For the gyro negative-peak
        // events (TO, HS) this is BACK-DATED to the peak sample (minus the
        // filter group delay), not the sample on which the event was declared.
        // For the other events it is the current sample's timestamp.
        int64_t event_timestamp_ns = 0;
    };

    GaitEvent check_state_transition(float gyro_z, float accel_norm, float foot_angle,
                                     int64_t timestamp_ns);

private:
    // Threshold-bracketed negative-peak detector. `threshold` is negative. The
    // detector arms when the (filtered) signal crosses DOWN past the threshold,
    // tracks the running minimum and the timestamp of that minimum sample, and
    // fires when the signal crosses back UP past the threshold. On firing it
    // reports the stored minimum-sample timestamp (the true peak) so the event
    // is back-dated relative to the firing sample.
    struct PeakDetector {
        bool armed = false;
        float min_value = 0.0f;
        int64_t min_timestamp_ns = 0;
        void reset() {
            armed = false;
            min_value = 0.0f;
            min_timestamp_ns = 0;
        }
    };
    static bool update_peak(PeakDetector& d, float value, int64_t ts, float threshold,
                            int64_t& out_peak_ts);

    float fs_;
    std::string foot_;
    GaitFSMState fsm_state_;

    float hs_threshold_ = -1.3963f;
    float ts_threshold_ = -0.5236f;
    float ho_threshold_ = -0.15f;
    float to_threshold_ = -3.4907f;
    float swing_gyro_threshold_ = 0.8727f;  // +50 deg/s
    float midstance_threshold_ = 3.0f;
    int min_swing_samples_ = 18;  // ~150 ms at 120 Hz

    // Moving-average filter state. ma_group_delay_ns_ = (window-1)/2 samples.
    int ma_window_ = 1;
    int64_t ma_group_delay_ns_ = 0;
    std::deque<float> gyro_ma_buffer_;
    std::deque<float> accel_ma_buffer_;

    // Negative-peak detectors for the back-dated TO and HS events.
    PeakDetector to_detector_;
    PeakDetector hs_detector_;

    // Per-swing dwell counter; reset on the ToeOff->Swing entry.
    int swing_samples_ = 0;

    // Previous filtered gyro_z, for the up-cross Toe-Strike detection.
    bool have_prev_gyro_ = false;
    float prev_gyro_ = 0.0f;

    // Filtered free-accel history for the midstance "quiet" window check.
    std::deque<float> accel_buffer_;

    static constexpr size_t BUFFER_SIZE = 200;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_GAIT_FSM_HPP
