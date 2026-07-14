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

namespace motorized_shoe {

// Slip perturbation controller. Drives the ELMO command node to deliver
// one-shot slip velocity bursts timed relative to gait events.
//
//   Mode AfterHS  -> backward slip. Fires `mode1_delay_after_hs_ms` after the
//                    next HS event. Unchanged.
//   Mode BeforeTO -> forward slip, now a PREDICTED, HS-anchored trigger. Toe-off
//                    detection is after-the-fact (the TO peak is only confirmed
//                    once gyro recovers), so we cannot react to TO and still fire
//                    BEFORE it. Instead, on the next HS (the reliable anchor) we
//                    schedule the slip for t_HS + max(0, stance_est - lead), where
//                    stance_est is a running average of measured (t_TO - t_HS)
//                    stance times. TO detection still runs (it defines the cycle
//                    and feeds stance_est) but is not the trigger.
//
// A stance estimator runs every tick, independent of arming, consuming HS/TO
// events to keep stance_est (and an HS->HS period for warm-up) up to date.
//
// Single-threaded: tick() runs in the main control loop. request_slip() is
// safe to call from another thread (keyboard handler).
class SlipPerturbationNode {
public:
    enum class Mode {
        None = 0,
        AfterHS = 1,
        BeforeTO = 2,
    };

    SlipPerturbationNode(const SlipConfig& cfg, DataBus& bus, SendCanCommandToElmoNode& cmd_node);

    void tick();
    void request_slip(Mode mode);

    bool is_active() const;

private:
    enum class State {
        Idle,
        ArmedAfterHS,
        ArmedBeforeTO,
        DelayingBeforeSlip,
        Slipping,
    };

    void start_slip_now();
    void end_slip_now();
    // AfterHS path: fire `delay_ms` after the next `trigger_phase` event.
    void scan_for_trigger_event(const char* trigger_phase, int delay_ms);

    // Stance estimator: runs every tick, consumes new HS/TO events for cfg_.foot.
    void update_stance_estimator();
    // Current stance estimate (ns): average of the last few measured stance times,
    // else 0.60 * HS->HS period as a warm-up fallback, else 0 (not warmed yet).
    int64_t stance_estimate_ns() const;
    // Schedule the forward (BeforeTO) slip to fire just before the predicted TO,
    // anchored on the HS at time t_hs_ns. Returns false if stance_est isn't warm
    // (caller skips one cycle until a measurement exists).
    bool schedule_before_to(int64_t t_hs_ns);

    const SlipConfig cfg_;
    DataBus& bus_;
    SendCanCommandToElmoNode& cmd_node_;

    std::atomic<int> pending_request_{0};  // Mode value
    State state_ = State::Idle;
    Mode current_mode_ = Mode::None;
    std::chrono::steady_clock::time_point timer_deadline_{};
    uint32_t last_seen_detection_count_ = 0;
    bool detection_count_initialized_ = false;

    // detection_count of the HS the forward slip is currently scheduled on, so a
    // newer HS (cadence sped up / overshoot) can cancel and reschedule.
    uint32_t before_to_anchor_count_ = 0;

    // --- Stance estimator state (independent of arming) ---
    uint32_t est_cursor_ = 0;          // last event detection_count consumed
    bool est_pending_hs_ = false;      // an HS awaiting its TO to close a stance
    int64_t est_pending_hs_ts_ = 0;
    bool est_have_last_hs_ = false;
    int64_t est_last_hs_ts_ = 0;       // for the HS->HS period
    int64_t est_hs_to_hs_ns_ = 0;      // most recent HS->HS period (warm-up fallback)
    std::deque<int64_t> stance_samples_ns_;  // recent (t_TO - t_HS)
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP
