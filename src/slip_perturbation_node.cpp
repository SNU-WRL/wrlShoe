#include "motorized_shoe/slip_perturbation_node.hpp"

#include <iostream>

#include "motorized_shoe/imu_watchdog.hpp"

namespace motorized_shoe {

namespace {
StanceEstimatorConfig estimator_config(const SlipConfig& cfg) {
    StanceEstimatorConfig e;
    e.window = cfg.stance_est_window;
    e.warmup_cycles = cfg.stance_est_warmup_cycles;
    e.stance_min_ms = cfg.stance_min_ms;
    e.stance_max_ms = cfg.stance_max_ms;
    e.cycle_min_ms = cfg.cycle_min_ms;
    e.cycle_max_ms = cfg.cycle_max_ms;
    e.reset_gap_ms = cfg.stance_est_reset_gap_ms;
    return e;
}
}  // namespace

SlipPerturbationNode::SlipPerturbationNode(
    const SlipConfig& cfg, DataBus& bus, SendCanCommandToElmoNode& cmd_node)
    : cfg_(cfg), bus_(bus), cmd_node_(cmd_node), estimator_(estimator_config(cfg)) {}

void SlipPerturbationNode::reset_estimator(const char* reason) {
    if (estimator_had_data_) {
        std::cerr << "[slip] stance estimator reset (" << reason << "); needs "
                  << estimator_.warmup_cycles() << " clean cycles before BeforeTO can schedule\n";
    }
    estimator_.reset(reason);
    estimator_had_data_ = false;
}

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
        reset_estimator("emergency stop");
        if (state_ != State::Idle) {
            std::cerr << "[slip] aborted: emergency stop active\n";
            state_ = State::Idle;
            current_mode_ = Mode::None;
        }
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const SystemSnapshot s = bus_.snapshot();
    const GaitPhase& gait = (cfg_.foot == "Left") ? s.gait_left : s.gait_right;
    const IMUData& imu = (cfg_.foot == "Left") ? s.imu_left : s.imu_right;
    const ImuWatchdog watchdog(cfg_.imu_stale_ms);
    const bool imu_stale = watchdog.is_stale(imu, s.timestamp_ns);

    // Consume any pending keyboard request (only while idle; ignore otherwise).
    const int req = pending_request_.exchange(0, std::memory_order_acq_rel);
    if (req != 0) {
        const Mode mode = static_cast<Mode>(req);
        const char* mode_name = (mode == Mode::AfterHS) ? "AfterHS" : "BeforeTO";
        std::string why;
        if (state_ != State::Idle) {
            std::cerr << "[slip] request ignored: slip already in progress\n";
        } else if (imu_stale) {
            // A dead slip-foot IMU means no HS will ever arrive: say so now
            // instead of sitting armed forever (2026-09-08 run).
            std::cerr << "[slip] REFUSED to arm " << mode_name << ": " << cfg_.foot
                      << " IMU is stale (" << (imu.valid ? (s.timestamp_ns - imu.timestamp_ns) / 1000000 : -1)
                      << " ms old) -- check the sensor/cable\n";
        } else if (!cmd_node_.is_drive_available(cfg_.foot, &why)) {
            std::cerr << "[slip] REFUSED to arm " << mode_name << ": " << cfg_.foot << " " << why << '\n';
        } else {
            // Seed the detection counter so we only react to the NEXT event,
            // not whatever phase happens to be current at arming time.
            last_seen_detection_count_ = gait.detection_count;
            detection_count_initialized_ = true;
            current_mode_ = mode;
            state_ = (mode == Mode::AfterHS) ? State::ArmedAfterHS : State::ArmedBeforeTO;
            std::cout << "[slip] armed mode=" << mode_name << " foot=" << cfg_.foot;
            if (mode == Mode::BeforeTO) {
                std::cout << " (" << estimator_.describe() << ")";
            }
            std::cout << '\n';
            std::cout.flush();
        }
    }

    const bool fault_active =
        (cfg_.foot == "Left") ? (s.status_left.valid && s.status_left.fault)
                              : (s.status_right.valid && s.status_right.fault);

    if (fault_active) {
        reset_estimator("drive fault");
    }

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

    // Disarm if the slip foot's IMU dies while we are waiting for a gait
    // event; a late-returning sensor must not fire a surprise slip.
    if (imu_stale && (state_ == State::ArmedAfterHS || state_ == State::ArmedBeforeTO ||
                      state_ == State::DelayingBeforeSlip)) {
        std::cerr << "[slip] DISARMED: " << cfg_.foot << " IMU went stale while armed\n";
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
                    before_to_anchor_ts_ = ev.timestamp_ns;
                    state_ = State::DelayingBeforeSlip;
                    scheduled = true;
                } else if (skipped_hs_logged_ != ev.detection_count) {
                    skipped_hs_logged_ = ev.detection_count;
                    std::cout << "[slip] BeforeTO: skipping HS (count=" << ev.detection_count
                              << "), estimator " << estimator_.describe() << '\n';
                    std::cout.flush();
                }
            });
        if (scheduled) {
            const int64_t stance_ns = estimator_.predict_stance_ns();
            std::cout << "[slip] BeforeTO HS anchor (count=" << before_to_anchor_count_
                      << "); predicted stance=" << (stance_ns / 1000000) << " ms, lead="
                      << cfg_.to_slip_lead_ms << " ms -> firing at HS+"
                      << ((stance_ns / 1000000) - cfg_.to_slip_lead_ms) << " ms ("
                      << estimator_.describe() << ")\n";
            std::cout.flush();
        }
    }

    // Forward slip reschedule: if a newer HS arrives before we fire (cadence
    // sped up / overshoot), cancel and re-anchor on it rather than firing late.
    if (state_ == State::DelayingBeforeSlip && current_mode_ == Mode::BeforeTO) {
        bool cancelled = false;
        bus_.for_each_gait_event_since(
            cfg_.foot, before_to_anchor_count_, [&](const GaitPhase& ev) {
                if (cancelled) return;
                if (ev.phase == "TO" && ev.timestamp_ns > before_to_anchor_ts_) {
                    // The real toe-off came before the predicted one: firing
                    // now would land with the foot already in the air. Abort
                    // and report it as a missed trial instead.
                    const auto to_time = std::chrono::steady_clock::time_point(
                        std::chrono::nanoseconds(ev.timestamp_ns));
                    const auto early_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                              timer_deadline_ - to_time)
                                              .count();
                    std::cerr << "[slip] BeforeTO CANCELLED: toe-off detected " << early_ms
                              << " ms before the scheduled fire (stance shorter than predicted)"
                              << " -- missed trial, press '" << cfg_.mode2_key << "' again\n";
                    cancelled = true;
                    return;
                }
                if (ev.phase == "RESET") {
                    std::cerr << "[slip] BeforeTO CANCELLED: gait FSM resync\n";
                    cancelled = true;
                    return;
                }
                if (ev.phase != "HS") return;
                if (schedule_before_to(ev.timestamp_ns)) {
                    before_to_anchor_count_ = ev.detection_count;
                    before_to_anchor_ts_ = ev.timestamp_ns;
                    std::cout << "[slip] BeforeTO rescheduled on newer HS (count="
                              << before_to_anchor_count_ << ")\n";
                    std::cout.flush();
                }
            });
        if (cancelled) {
            state_ = State::Idle;
            current_mode_ = Mode::None;
            return;
        }
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
            std::string note;
            if (ev.phase == "HS") {
                note = estimator_.on_heel_strike(ev.timestamp_ns);
                estimator_had_data_ = true;
            } else if (ev.phase == "TO") {
                note = estimator_.on_toe_off(ev.timestamp_ns);
            } else if (ev.phase == "RESET") {
                reset_estimator("gait FSM resync");
            }
            if (!note.empty()) {
                std::cout << "[slip] stance estimator: " << note << '\n';
                std::cout.flush();
            }
        });
}

bool SlipPerturbationNode::schedule_before_to(int64_t t_hs_ns) {
    const int64_t stance_ns = estimator_.predict_stance_ns();
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
