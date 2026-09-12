#ifndef MOTORIZED_SHOE_STANCE_ESTIMATOR_HPP
#define MOTORIZED_SHOE_STANCE_ESTIMATOR_HPP

#include <cstdint>
#include <deque>
#include <string>

namespace motorized_shoe {

struct StanceEstimatorConfig {
    int window = 3;            // clean cycles kept for the medians
    int warmup_cycles = 1;     // clean stances required before predicting
    int stance_min_ms = 300;   // plausibility window on a measured stance
    int stance_max_ms = 1500;
    int cycle_min_ms = 600;    // plausibility window on the HS->HS period
    int cycle_max_ms = 2000;
    int reset_gap_ms = 2500;   // HS->HS gap that means "walking stopped"
    // The prediction multiplies the median stance fraction by a cycle
    // period. Using the single latest period passes its jitter straight
    // into the prediction, so the median of the recent clean periods is used
    // unless the latest one differs from it by more than this percentage
    // (a real cadence change), in which case the latest period wins.
    int cadence_change_pct = 15;
};

// Predicts the stance duration (HS -> TO) of the cycle that just started, for
// the HS-anchored forward slip.
//
// Robustness rules (2026-09-13), motivated by the Sep-2026 logs where pauses
// produced 6-32 s "stances" and false events produced 90-350 ms ones, and a
// mean of the last two samples then scheduled the slip seconds late or in
// mid-stance:
//   1. A stance sample is accepted only inside [stance_min, stance_max] and
//      only if the HS->HS period enclosing it is inside [cycle_min, cycle_max].
//   2. The prediction is the median stance FRACTION (stance / cycle) of the
//      last `window` clean cycles times the period of the cycle that just
//      completed, so it follows cadence changes. Until a fraction exists it
//      falls back to the median absolute stance.
//   3. (caller) A toe-off arriving before the scheduled fire cancels the slip.
//   4. The estimator resets on an HS gap > reset_gap, on an FSM resync event,
//      and on request (drive fault, emergency stop), and needs
//      `warmup_cycles` fresh clean stances before it predicts again.
class StanceEstimator {
public:
    explicit StanceEstimator(const StanceEstimatorConfig& cfg = StanceEstimatorConfig{});

    // Feed gait events in time order (timestamps in ns, back-dated to the
    // gyro trough). Returns a short note when something notable happened
    // (sample rejected, reset), empty otherwise -- for console logging.
    std::string on_heel_strike(int64_t t_hs_ns);
    std::string on_toe_off(int64_t t_to_ns);
    void reset(const char* reason);

    bool warm() const;
    int clean_stances() const { return static_cast<int>(stances_ns_.size()); }
    int clean_fractions() const { return static_cast<int>(fractions_.size()); }
    int warmup_cycles() const { return cfg_.warmup_cycles; }

    // Predicted stance (ns) for the cycle beginning at the most recent HS;
    // 0 when not warm.
    int64_t predict_stance_ns() const;
    // Period of the most recently completed plausible cycle (ns), 0 if none.
    int64_t last_cycle_ns() const { return last_cycle_ns_; }
    // Period used for the prediction (smoothed unless cadence changed).
    int64_t cycle_for_prediction_ns() const;
    // Human-readable state for log lines.
    std::string describe() const;

private:
    StanceEstimatorConfig cfg_;
    std::deque<int64_t> stances_ns_;   // accepted absolute stances
    std::deque<double> fractions_;     // accepted stance / cycle
    std::deque<int64_t> periods_ns_;   // accepted HS->HS periods
    bool have_last_hs_ = false;
    int64_t last_hs_ns_ = 0;
    bool stance_open_ = false;         // HS seen, waiting for its TO
    int64_t open_stance_ns_ = 0;       // stance measured for the current cycle (-1 = none/rejected)
    int64_t last_cycle_ns_ = 0;
    std::string last_reset_reason_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_STANCE_ESTIMATOR_HPP
