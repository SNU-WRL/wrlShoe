#ifndef MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP
#define MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

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
// A request can override the YAML velocity, delay and duration (the schedule
// app does, per slip). Requests with an id > 0 report what happened to them
// through take_outcomes(); keyboard requests (id 0) only print.
//
// Single-threaded: tick() runs in the main control loop. request_slip() is
// safe to call from another thread (keyboard handler); cancel() and
// take_outcomes() must be called from the control-loop thread.
class SlipPerturbationNode {
public:
    enum class Mode {
        None = 0,
        AfterHS = 1,
        LateStance = 2,
        MidStance = 3,
    };

    enum class State {
        Idle = 0,
        Armed = 1,
        DelayingBeforeSlip = 2,
        Slipping = 3,
    };

    struct Request {
        Mode mode = Mode::None;
        int32_t velocity = 0;   // signed counts/s (+ = backward, - = forward)
        int delay_ms = 0;       // after the trigger event
        int duration_ms = 0;
        int id = 0;             // > 0: outcomes are reported with this id
    };

    struct Outcome {
        enum class Kind {
            Armed,      // waiting for the trigger event
            Triggered,  // trigger event seen; event_ns = its timestamp
            Started,    // velocity commanded
            Completed,  // burst over, velocity 0 commanded
            Refused,    // never armed (busy, IMU stale, drive unavailable, e-stop, no FF/HO)
            Cancelled,  // stance ended before the fire time, or cancel()
            Disarmed,   // slip-foot IMU went stale while armed
            Aborted,    // drive fault or e-stop while active (see during_slip)
        };
        int id = 0;
        Kind kind = Kind::Armed;
        int64_t time_ns = 0;     // steady clock, same base as the log
        int64_t event_ns = 0;    // Triggered: gait event timestamp
        bool during_slip = false;  // Aborted: the burst had already started
        std::string detail;
    };

    SlipPerturbationNode(const SlipConfig& cfg, DataBus& bus, SendCanCommandToElmoNode& cmd_node);

    void tick();
    // Keyboard path: velocity, delay and duration from the YAML.
    void request_slip(Mode mode);
    void request_slip(const Request& request);
    // Drops a pending request and disarms an armed / delaying slip (reported
    // as Cancelled with `reason`). A slip already running is not touched;
    // returns false in that case.
    bool cancel(const std::string& reason);
    std::vector<Outcome> take_outcomes();

    bool is_active() const;
    State state() const { return state_; }
    Mode current_mode() const { return current_.mode; }
    const std::string& foot() const { return cfg_.foot; }

    static const char* mode_name(Mode mode);
    static const char* outcome_name(Outcome::Kind kind) {
        switch (kind) {
            case Outcome::Kind::Armed: return "armed";
            case Outcome::Kind::Triggered: return "triggered";
            case Outcome::Kind::Started: return "started";
            case Outcome::Kind::Completed: return "completed";
            case Outcome::Kind::Refused: return "refused";
            case Outcome::Kind::Cancelled: return "cancelled";
            case Outcome::Kind::Disarmed: return "disarmed";
            case Outcome::Kind::Aborted: return "aborted";
        }
        return "?";
    }
    // YAML defaults for a mode: signed velocity and trigger delay.
    int32_t default_velocity(Mode mode) const;
    int default_delay_ms(Mode mode) const;

private:

    void start_slip_now();
    void end_slip_now();
    // AfterHS path: fire `delay_ms` after the next `trigger_phase` event.
    void scan_for_trigger_event(const char* trigger_phase, int delay_ms);

    // Stance estimator (diagnostic): runs every tick, consumes new HS/TO/RESET
    // events for cfg_.foot (see stance_estimator.hpp for the robustness rules).
    void update_stance_estimator();
    void reset_estimator(const char* reason);

    const char* trigger_label(Mode mode) const;   // "HS" / "FF" / "HO"
    char mode_key(Mode mode) const;
    void report(Outcome::Kind kind, const std::string& detail = "", int64_t event_ns = 0,
                bool during_slip = false, int id = -1);
    void go_idle();

    const SlipConfig cfg_;
    DataBus& bus_;
    SendCanCommandToElmoNode& cmd_node_;

    std::mutex pending_mutex_;
    std::optional<Request> pending_request_;
    State state_ = State::Idle;
    Request current_;
    std::vector<Outcome> outcomes_;
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
