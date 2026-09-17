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

    // Contact heel-strike mode (default off = the trough-recovery detector above).
    // Motivation (2026-09-15 logs, 24 AfterHS slips): the trough detector
    // declares HS when the filtered gyro climbs back above hs_threshold AFTER
    // the foot-slap trough, i.e. at foot-flat, median 250 ms after the first
    // ground contact; and because state parity is the only thing telling an HS
    // trough from a TO trough, one weak trough inverts the machine until a >2 s
    // pause (15% of HS events, 5 of 24 slips fired at toe-off).
    // When enabled:
    //   * HS needs swing evidence first: raw gyro_z > swing_gyro_min (rad/s)
    //     for swing_min_ms. A push-off trough is preceded by a flat foot, so it
    //     can never be taken for a heel strike. Swing evidence seen while the
    //     machine still says Stance (missed toe-off) moves it to Swing with a
    //     "SWING" event instead of leaving it inverted.
    //   * HS then fires, stamped with the firing sample's own time, on the
    //     first of (a) a contact impact: |a_k - a_(k-1)| >= jerk_threshold
    //     (m/s^2 per sample; 10-50 at contact vs 1-3 in swing, unlike the
    //     |accel| norm that swing shake exceeds) once gyro_z has fallen
    //     jerk_gyro_drop below its swing peak and jerk_holdoff_ms after the
    //     swing evidence, or (b) raw gyro_z < 0 (start of the foot-down
    //     rotation). min_swing_dwell_ms and the accel veto are not used.
    //   * The TO trough detector and the swing-evidence counter only arm once
    //     the stance has settled: |filtered gyro_z| < flat_gyro_max for
    //     flat_min_ms, or settle_timeout_ms after HS. HS now precedes the slap
    //     trough, which would otherwise be taken for the toe-off.
    //   * Swing with a flat foot for flat_reset_ms means the landing was
    //     missed (weak step): drop to Stance with a "RESET" event.
    struct ContactHsParams {
        float swing_gyro_min = 1.5f;
        int swing_min_ms = 50;
        float jerk_threshold = 8.0f;
        float jerk_gyro_drop = 0.5f;
        int jerk_holdoff_ms = 100;
        float flat_gyro_max = 0.5f;
        int flat_min_ms = 100;
        int settle_timeout_ms = 400;
        int flat_reset_ms = 200;
    };
    void set_contact_hs(bool enabled, const ContactHsParams& params);

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
        // "TO" or "HS" on the firing sample, "RESET" when the resync guard
        // dropped the machine back to Stance, "SWING" (contact-HS mode only) when
        // swing evidence moved it to Swing without a toe-off event (no usable
        // TO time; consumers ignore it), "" otherwise. The slip node
        // consumes these labels directly (HS = backward/AfterHS anchor + stance
        // estimator; TO = stance estimator only; RESET = estimator reset).
        const char* event_label = "";
    };

    // accel_* is the RAW accelerometer vector in m/s^2 (gravity NOT removed).
    GaitEvent check_state_transition(float gyro_z, float accel_x, float accel_y, float accel_z,
                                     float foot_angle, int64_t timestamp_ns);

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

    // Contact heel-strike mode (see set_contact_hs).
    GaitEvent step_contact_hs(GaitEvent event, float gyro_raw, float gyro_f, float jerk,
                            int64_t timestamp_ns);
    void enter_swing_contact();
    static int ms_to_samples(int ms, float fs);
    bool contact_hs_ = false;
    ContactHsParams contact_;
    int swing_run_needed_ = 5;
    int jerk_holdoff_samples_ = 10;
    int flat_min_samples_ = 10;
    int settle_timeout_samples_ = 40;
    int flat_reset_samples_ = 20;
    bool have_prev_accel_ = false;
    float prev_accel_[3] = {0.0f, 0.0f, 0.0f};
    int swing_gyro_run_ = 0;        // consecutive samples above swing_gyro_min
    bool swing_evidence_ = false;   // this Swing has shown real swing rotation
    int evidence_samples_ = 0;      // samples since the evidence was established
    float swing_peak_ = 0.0f;       // largest raw gyro_z since the evidence
    bool stance_settled_ = true;    // foot-flat seen (or timed out) since HS
    int flat_run_ = 0;              // consecutive |filtered gyro_z| < flat_gyro_max
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_GAIT_FSM_HPP
