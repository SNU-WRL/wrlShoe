#ifndef MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP
#define MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"
#include "motorized_shoe/send_can_command_to_elmo_node.hpp"
#include "motorized_shoe/stance_estimator.hpp"

namespace motorized_shoe {

// Slip perturbation controller. Drives the ELMO command node to deliver
// one-shot slip velocity bursts timed relative to gait events of the slip foot:
//
//   Mode AfterHS    (key mode1) -> backward (+velocity) slip,
//                    `mode1_delay_after_hs_ms` after the next HS (early stance).
//   Mode MidStance  (key mode3) -> slip `mid_stance_delay_ms` after the next
//                    foot-flat (FF) event, direction `mid_stance_slip_direction`.
//   Mode LateStance (key mode2) -> forward (-velocity) slip
//                    `late_stance_delay_ms` after the next heel-off (HO) event.
//
// All three react to a detected event; nothing is predicted. FF and HO come
// from the gait FSM (contact HS mode with gait_detection.stance_events on);
// modes 2/3 refuse to arm without them. A mid/late-stance slip still waiting
// for its deadline is cancelled if the stance ends first (TO, HS or RESET).
//
// The stance estimator still runs every tick as a DIAGNOSTIC only: when a slip
// fires its predicted stance is printed, and at the next toe-off the measured
// stance is printed next to it.
//
// Single-threaded: tick() runs in the main control loop. request_slip() is
// safe to call from another thread (keyboard handler).
class SlipPerturbationNode {
public:
    enum class Mode {
        None = 0,
        AfterHS = 1,
        LateStance = 2,
        MidStance = 3,
    };

    SlipPerturbationNode(const SlipConfig& cfg, DataBus& bus, SendCanCommandToElmoNode& cmd_node);

    void tick();
    void request_slip(Mode mode);

    bool is_active() const;

private:
    enum class State {
        Idle,
        Armed,
        DelayingBeforeSlip,
        Slipping,
    };

    void start_slip_now();
    void end_slip_now();
    // AfterHS path: fire `delay_ms` after the next `trigger_phase` event.
    void scan_for_trigger_event(const char* trigger_phase, int delay_ms);

    // Stance estimator (diagnostic): runs every tick, consumes new HS/TO/RESET
    // events for cfg_.foot (see stance_estimator.hpp for the robustness rules).
    void update_stance_estimator();
    void reset_estimator(const char* reason);

    static const char* mode_name(Mode mode);
    const char* trigger_label(Mode mode) const;   // "HS" / "FF" / "HO"
    int trigger_delay_ms(Mode mode) const;
    char mode_key(Mode mode) const;
    int32_t slip_velocity_for(Mode mode) const;

    const SlipConfig cfg_;
    DataBus& bus_;
    SendCanCommandToElmoNode& cmd_node_;

    std::atomic<int> pending_request_{0};  // Mode value
    State state_ = State::Idle;
    Mode current_mode_ = Mode::None;
    std::chrono::steady_clock::time_point timer_deadline_{};
    uint32_t last_seen_detection_count_ = 0;
    bool detection_count_initialized_ = false;

    // --- Stance estimator state (diagnostic, independent of arming) ---
    uint32_t est_cursor_ = 0;          // last event detection_count consumed
    StanceEstimator estimator_;
    bool estimator_had_data_ = false;  // suppress repeated reset messages
    int64_t est_last_hs_ns_ = 0;       // latest HS seen by the estimator
    int64_t est_pred_at_hs_ns_ = 0;    // its predicted stance at that HS (0 = not warm)
    bool diag_pending_ = false;        // a slip fired; report predicted vs measured at TO
    int64_t diag_hs_ns_ = 0;
    int64_t diag_pred_ns_ = 0;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP
