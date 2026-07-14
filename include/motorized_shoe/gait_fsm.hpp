#ifndef MOTORIZED_SHOE_GAIT_FSM_HPP
#define MOTORIZED_SHOE_GAIT_FSM_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace motorized_shoe {

// Minimal two-state gait cycle, driven purely by gyro negative-peak detection:
//   Stance -> waiting for the toe-off (TO) negative gyro peak.
//   Swing  -> waiting for the heel-strike (HS) negative gyro peak.
// The old six-state machine (HeelOff/ToeOff/HeelStrike/ToeStrike/MidStance) and
// its accel-quiet midstance gate were removed: the midstance gate starved on the
// gravity-removal residual and jammed the cycle in ToeStrike, which in turn
// starved the MSt-gated forward slip. Transitions are now pure gyro; free-accel /
// gravity estimation is no longer on the critical path.
enum class GaitState {
    Stance,
    Swing
};

const char* gait_state_to_string(GaitState state);

class GaitEventFSM {
public:
    GaitEventFSM(float fs = 120.0f, const std::string& foot = "Right");

    // hs / to are the (negative) gyro_z peak thresholds in rad/s. min_swing_dwell_ms
    // gates the HS detector so it cannot latch onto an early-swing dip.
    void set_thresholds(float hs, float to, int min_swing_dwell_ms);

    // Per-state resync: if a state's dwell exceeds max(state_timeout_ms,
    // ~1.5x the most recent HS->HS cycle period) with no event, both peak
    // detectors and the cycle clocks are reset and the machine drops to Stance,
    // so a single missed gyro peak cannot stall it.
    void set_state_timeout_ms(int ms);

    // Optional HS accept gate: only declare a heel strike if a RAW |accel| impact
    // spike exceeded impact_threshold (m/s^2) during the swing dwell. Uses the raw
    // accel norm (the impact is ~30-40 m/s^2, gravity negligible), NOT the
    // gravity-removed free accel, so it does not reinherit gravity-calibration
    // fragility. Default off.
    void set_hs_accel_veto(bool enabled, float impact_threshold);

    // Moving-average window (samples) on gyro_z. window <= 1 disables filtering.
    // The (window-1)/2-sample group delay is compensated for when back-dating
    // event timestamps.
    void set_filter_window(int window);

    struct GaitEvent {
        GaitState state = GaitState::Stance;
        float gyro_z_value = 0.0f;
        uint32_t detection_count = 0;
        bool event_detected = false;
        // Timestamp the event is attributed to. For both gyro negative-peak
        // events (TO, HS) this is BACK-DATED to the trough sample (minus the
        // filter group delay), not the sample on which the event was declared.
        int64_t event_timestamp_ns = 0;
        // "TO" or "HS" on the firing sample, "" otherwise. The slip node consumes
        // these labels directly (HS = backward/AfterHS anchor + stance estimator;
        // TO = stance estimator only).
        const char* event_label = "";
    };

    GaitEvent check_state_transition(float gyro_z, float raw_accel_norm, float foot_angle,
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
    GaitState current_state_ = GaitState::Stance;
    uint32_t detection_count_ = 0;

    float hs_threshold_ = -1.3963f;
    float to_threshold_ = -3.4907f;
    int min_swing_samples_ = 18;  // ~150 ms at 120 Hz

    // Resync timeout (samples) and recent cycle clock.
    int state_timeout_samples_ = 240;  // ~2000 ms at 120 Hz
    int state_dwell_samples_ = 0;
    int64_t last_hs_ts_ns_ = 0;
    int64_t recent_cycle_ns_ = 0;  // most recent HS->HS period
    bool have_last_hs_ = false;

    // Optional raw-accel impact gate on HS.
    bool hs_accel_veto_ = false;
    float hs_impact_threshold_ = 20.0f;
    bool impact_seen_in_swing_ = false;

    // Moving-average filter state. ma_group_delay_ns_ = (window-1)/2 samples.
    int ma_window_ = 1;
    int64_t ma_group_delay_ns_ = 0;
    std::deque<float> gyro_ma_buffer_;

    // Negative-peak detectors for the back-dated TO and HS events.
    PeakDetector to_detector_;
    PeakDetector hs_detector_;

    // Per-swing dwell counter; reset on the Stance->Swing (TO) transition.
    int swing_samples_ = 0;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_GAIT_FSM_HPP
