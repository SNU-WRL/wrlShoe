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

    // Keep the stance estimate fresh every tick, independent of arming, so a
    // BeforeTO arm can schedule against a warm estimate immediately.
    update_stance_estimator();

    // Detection step. May set state_ = DelayingBeforeSlip with a deadline that
    // is already in the past (the gait event happened a few ms before this
    // tick); we then want to start the slip in the SAME tick rather than
    // waiting for the next one.
    if (state_ == State::ArmedAfterHS) {
        scan_for_trigger_event("HS", cfg_.mode1_delay_after_hs_ms);
    } else if (state_ == State::ArmedBeforeTO) {
        // Forward slip: on the NEXT HS after arming, schedule the slip to land
        // just before the predicted toe-off.
        bool scheduled = false;
        bus_.for_each_gait_event_since(
            cfg_.foot, last_seen_detection_count_, [&](const GaitPhase& ev) {
                if (scheduled) return;
                last_seen_detection_count_ = ev.detection_count;
                if (ev.phase != "HS") return;
                if (schedule_before_to(ev.timestamp_ns)) {
                    before_to_anchor_count_ = ev.detection_count;
                    state_ = State::DelayingBeforeSlip;
                    scheduled = true;
                }
                // If stance_est isn't warm yet schedule_before_to() returns false;
                // skip this HS and wait for the next one (the estimator warms after
                // one completed HS->TO stance).
            });
        if (scheduled) {
            const int64_t stance_ns = stance_estimate_ns();
            std::cout << "[slip] BeforeTO HS anchor (count=" << before_to_anchor_count_
                      << "); stance_est=" << (stance_ns / 1000000) << " ms, lead="
                      << cfg_.to_slip_lead_ms << " ms -> firing before predicted TO\n";
            std::cout.flush();
        }
    }

    // Forward slip reschedule: if a newer HS arrives before we fire (cadence
    // sped up / overshoot), cancel and re-anchor on it rather than firing late.
    if (state_ == State::DelayingBeforeSlip && current_mode_ == Mode::BeforeTO) {
        bus_.for_each_gait_event_since(
            cfg_.foot, before_to_anchor_count_, [&](const GaitPhase& ev) {
                if (ev.phase != "HS") return;
                if (schedule_before_to(ev.timestamp_ns)) {
                    before_to_anchor_count_ = ev.detection_count;
                    std::cout << "[slip] BeforeTO rescheduled on newer HS (count="
                              << before_to_anchor_count_ << ")\n";
                    std::cout.flush();
                }
            });
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

void SlipPerturbationNode::update_stance_estimator() {
    bus_.for_each_gait_event_since(
        cfg_.foot, est_cursor_, [&](const GaitPhase& ev) {
            est_cursor_ = ev.detection_count;
            if (ev.phase == "HS") {
                if (est_have_last_hs_) {
                    const int64_t period = ev.timestamp_ns - est_last_hs_ts_;
                    if (period > 0) {
                        est_hs_to_hs_ns_ = period;
                    }
                }
                est_last_hs_ts_ = ev.timestamp_ns;
                est_have_last_hs_ = true;
                // Open a stance interval; the matching TO closes it.
                est_pending_hs_ = true;
                est_pending_hs_ts_ = ev.timestamp_ns;
            } else if (ev.phase == "TO") {
                if (est_pending_hs_) {
                    const int64_t stance = ev.timestamp_ns - est_pending_hs_ts_;
                    if (stance > 0) {
                        stance_samples_ns_.push_back(stance);
                        const size_t win = (cfg_.stance_est_window > 0)
                                               ? static_cast<size_t>(cfg_.stance_est_window)
                                               : 1;
                        while (stance_samples_ns_.size() > win) {
                            stance_samples_ns_.pop_front();
                        }
                    }
                    est_pending_hs_ = false;
                }
            }
        });
}

int64_t SlipPerturbationNode::stance_estimate_ns() const {
    if (!stance_samples_ns_.empty()) {
        int64_t sum = 0;
        for (int64_t v : stance_samples_ns_) {
            sum += v;
        }
        return sum / static_cast<int64_t>(stance_samples_ns_.size());
    }
    if (est_hs_to_hs_ns_ > 0) {
        // Warm-up fallback before any stance has been measured: stance is ~0.60
        // of the HS->HS gait period.
        return static_cast<int64_t>(0.60 * static_cast<double>(est_hs_to_hs_ns_));
    }
    return 0;  // not warmed yet
}

bool SlipPerturbationNode::schedule_before_to(int64_t t_hs_ns) {
    const int64_t stance_ns = stance_estimate_ns();
    if (stance_ns <= 0) {
        return false;
    }
    const int64_t lead_ns = static_cast<int64_t>(cfg_.to_slip_lead_ms) * 1000000LL;
    int64_t delay_ns = stance_ns - lead_ns;
    if (delay_ns < 0) {
        delay_ns = 0;
    }
    const int64_t fire_ns = t_hs_ns + delay_ns;
    timer_deadline_ =
        std::chrono::steady_clock::time_point(std::chrono::nanoseconds(fire_ns));
    return true;
}

void SlipPerturbationNode::start_slip_now() {
    // BeforeTO slips push the foot in the opposite direction of an HS-triggered
    // slip — the perturbation simulates a foot slipping forward at push-off, the
    // inverse of the heel-strike slip backward. The onset is PREDICTED from the
    // HS anchor (stance_est - to_slip_lead_ms) because TO detection is
    // after-the-fact and cannot be reacted to before toe-off actually happens.
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
