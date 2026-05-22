#include "motorized_shoe/slip_perturbation_node.hpp"

#include <iostream>

namespace motorized_shoe {

SlipPerturbationNode::SlipPerturbationNode(
    const SlipConfig& cfg, DataBus& bus, SendCanCommandToElmoNode& cmd_node)
    : cfg_(cfg), bus_(bus), cmd_node_(cmd_node) {}

void SlipPerturbationNode::request_slip(Mode mode) {
    pending_request_.store(static_cast<int>(mode), std::memory_order_release);
}

bool SlipPerturbationNode::is_active() const {
    return state_ != State::Idle;
}

void SlipPerturbationNode::tick() {
    if (!cfg_.enabled) {
        return;
    }

    // Emergency stop trumps everything. Drop any pending request, force the
    // state machine back to Idle, and let the command node hold the motors at 0.
    if (cmd_node_.is_emergency_stopped()) {
        pending_request_.store(0, std::memory_order_release);
        if (state_ != State::Idle) {
            std::cerr << "[slip] aborted: emergency stop active\n";
            state_ = State::Idle;
            current_mode_ = Mode::None;
        }
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    // Consume any pending keyboard request (only while idle; ignore otherwise).
    const int req = pending_request_.exchange(0, std::memory_order_acq_rel);
    if (req != 0) {
        if (state_ == State::Idle) {
            const Mode mode = static_cast<Mode>(req);
            const SystemSnapshot s = bus_.snapshot();
            const GaitPhase& gait =
                (cfg_.foot == "Left") ? s.gait_left : s.gait_right;
            // Seed the detection counter so we only react to the NEXT event,
            // not whatever phase happens to be current at arming time.
            last_seen_detection_count_ = gait.detection_count;
            detection_count_initialized_ = true;
            current_mode_ = mode;
            state_ = (mode == Mode::AfterHS) ? State::ArmedAfterHS : State::ArmedBeforeTO;
            std::cout << "[slip] armed mode="
                      << (mode == Mode::AfterHS ? "AfterHS" : "BeforeTO")
                      << " foot=" << cfg_.foot << '\n';
            std::cout.flush();
        } else {
            std::cerr << "[slip] request ignored: slip already in progress\n";
        }
    }

    const SystemSnapshot s = bus_.snapshot();
    const bool fault_active =
        (cfg_.foot == "Left") ? (s.status_left.valid && s.status_left.fault)
                              : (s.status_right.valid && s.status_right.fault);

    // Abort on fault during any active phase.
    if (fault_active && state_ != State::Idle) {
        if (state_ == State::Slipping || cmd_node_.is_externally_controlled(cfg_.foot)) {
            cmd_node_.release_external_control(cfg_.foot);
        }
        std::cerr << "[slip] aborted: fault on " << cfg_.foot << '\n';
        state_ = State::Idle;
        current_mode_ = Mode::None;
        return;
    }

    // Detection step. May set state_ = DelayingBeforeSlip with a deadline that
    // is already in the past (the gait event happened a few ms before this
    // tick); we then want to start the slip in the SAME tick rather than
    // waiting for the next one.
    if (state_ == State::ArmedAfterHS) {
        scan_for_trigger_event("HS", cfg_.mode1_delay_after_hs_ms);
    } else if (state_ == State::ArmedBeforeTO) {
        scan_for_trigger_event("HO", cfg_.mode2_delay_after_ho_ms);
    }

    // Deadline check. Fall through across states so a delay_ms of 0 (or a
    // deadline already in the past) fires immediately instead of costing
    // one extra slip tick per transition.
    if (state_ == State::DelayingBeforeSlip && now >= timer_deadline_) {
        start_slip_now();
        timer_deadline_ = now + std::chrono::milliseconds(cfg_.slip_duration_ms);
        state_ = State::Slipping;
    }
    if (state_ == State::Slipping && now >= timer_deadline_) {
        end_slip_now();
        state_ = State::Idle;
        current_mode_ = Mode::None;
    }
}

void SlipPerturbationNode::scan_for_trigger_event(
    const char* trigger_phase, int delay_ms) {
    // Walk the buffered transitions in order. The buffer captures every FSM
    // state change with its true timestamp, so we never miss the entry into
    // the trigger phase even if a slip tick is delayed past it. The delay
    // timer is anchored to the event's actual timestamp, not to slip-tick
    // time, which keeps the slip onset aligned with the gait event.
    bool fired = false;
    bus_.for_each_gait_event_since(
        cfg_.foot, last_seen_detection_count_,
        [&](const GaitPhase& ev) {
            if (fired) return;
            last_seen_detection_count_ = ev.detection_count;
            if (ev.phase == trigger_phase) {
                const auto event_time = std::chrono::steady_clock::time_point(
                    std::chrono::nanoseconds(ev.timestamp_ns));
                timer_deadline_ = event_time + std::chrono::milliseconds(delay_ms);
                fired = true;
            }
        });
    if (fired) {
        state_ = State::DelayingBeforeSlip;
        std::cout << "[slip] " << trigger_phase
                  << " entry detected (count=" << last_seen_detection_count_
                  << "); firing in " << delay_ms << " ms from event time\n";
        std::cout.flush();
    }
}

void SlipPerturbationNode::start_slip_now() {
    // BeforeTO (HO-triggered) slips push the foot in the opposite direction
    // of an HS-triggered slip — the perturbation simulates a foot slipping
    // forward at push-off, the inverse of the heel-strike slip backward.
    const int32_t velocity = (current_mode_ == Mode::BeforeTO)
                                 ? -cfg_.slip_velocity
                                 : cfg_.slip_velocity;
    std::cout << "[slip] START foot=" << cfg_.foot
              << " mode=" << (current_mode_ == Mode::BeforeTO ? "BeforeTO" : "AfterHS")
              << " velocity=" << velocity
              << " duration_ms=" << cfg_.slip_duration_ms << '\n';
    std::cout.flush();
    cmd_node_.inject_velocity(cfg_.foot, velocity);
}

void SlipPerturbationNode::end_slip_now() {
    std::cout << "[slip] END foot=" << cfg_.foot << " -> commanding velocity 0\n";
    std::cout.flush();
    // Hold the foot at 0 after the slip. Using inject_velocity (not
    // release_external_control) keeps the external-control flag set so the
    // gait-phase velocity map does not immediately re-spin the motor; the
    // operator resumes normal gait by toggling emergency stop ('s' then 'r').
    cmd_node_.inject_velocity(cfg_.foot, 0);
}

}  // namespace motorized_shoe
