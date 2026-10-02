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
        std::cerr << "[slip] stance estimator (diagnostic) reset (" << reason << ")\n";
    }
    estimator_.reset(reason);
    estimator_had_data_ = false;
    est_pred_at_hs_ns_ = 0;
    diag_pending_ = false;
}

const char* SlipPerturbationNode::mode_name(Mode mode) {
    switch (mode) {
        case Mode::AfterHS: return "AfterHS";
        case Mode::MidStance: return "MidStance";
        case Mode::LateStance: return "LateStance";
        default: return "None";
    }
}

const char* SlipPerturbationNode::trigger_label(Mode mode) const {
    switch (mode) {
        case Mode::MidStance: return "FF";
        case Mode::LateStance: return "HO";
        default: return "HS";
    }
}

int SlipPerturbationNode::trigger_delay_ms(Mode mode) const {
    switch (mode) {
        case Mode::MidStance: return cfg_.mid_stance_delay_ms;
        case Mode::LateStance: return cfg_.late_stance_delay_ms;
        default: return cfg_.mode1_delay_after_hs_ms;
    }
}

char SlipPerturbationNode::mode_key(Mode mode) const {
    switch (mode) {
        case Mode::MidStance: return cfg_.mode3_key;
        case Mode::LateStance: return cfg_.mode2_key;
        default: return cfg_.mode1_key;
    }
}

int32_t SlipPerturbationNode::slip_velocity_for(Mode mode) const {
    // AfterHS pushes the foot backward (+), the late-stance slip forward (-);
    // the mid-stance direction is configurable.
    switch (mode) {
        case Mode::LateStance: return -cfg_.slip_velocity;
        case Mode::MidStance:
            return (cfg_.mid_stance_slip_direction == "backward") ? cfg_.slip_velocity
                                                                  : -cfg_.slip_velocity;
        default: return cfg_.slip_velocity;
    }
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
        const char* name = mode_name(mode);
        std::string why;
        if (state_ != State::Idle) {
            std::cerr << "[slip] request ignored: slip already in progress\n";
        } else if (imu_stale) {
            // A dead slip-foot IMU means no HS will ever arrive: say so now
            // instead of sitting armed forever (2026-09-08 run).
            std::cerr << "[slip] REFUSED to arm " << name << ": " << cfg_.foot
                      << " IMU is stale (" << (imu.valid ? (s.timestamp_ns - imu.timestamp_ns) / 1000000 : -1)
                      << " ms old) -- check the sensor/cable\n";
        } else if (!cmd_node_.is_drive_available(cfg_.foot, &why)) {
            std::cerr << "[slip] REFUSED to arm " << name << ": " << cfg_.foot << " " << why << '\n';
        } else if (mode != Mode::AfterHS && !cfg_.stance_events_available) {
            std::cerr << "[slip] REFUSED to arm " << name << ": the gait FSM emits no FF/HO events"
                      << " (needs gait_detection.hs_contact_detection and"
                      << " gait_detection.stance_events.enabled)\n";
        } else {
            // Seed the detection counter so we only react to the NEXT event,
            // not whatever phase happens to be current at arming time.
            last_seen_detection_count_ = gait.detection_count;
            detection_count_initialized_ = true;
            current_mode_ = mode;
            state_ = State::Armed;
            std::cout << "[slip] armed mode=" << name << " foot=" << cfg_.foot << ": fires "
                      << trigger_delay_ms(mode) << " ms after the next " << trigger_label(mode)
                      << ", velocity " << slip_velocity_for(mode) << '\n';
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
    if (imu_stale && (state_ == State::Armed || state_ == State::DelayingBeforeSlip)) {
        std::cerr << "[slip] DISARMED: " << cfg_.foot << " IMU went stale while armed\n";
        state_ = State::Idle;
        current_mode_ = Mode::None;
        return;
    }

    // Keep the (diagnostic) stance estimate fresh every tick.
    update_stance_estimator();

    // Detection step. May set state_ = DelayingBeforeSlip with a deadline that
    // is already in the past (the gait event happened a few ms before this
    // tick); we then want to start the slip in the SAME tick rather than
    // waiting for the next one.
    if (state_ == State::Armed) {
        scan_for_trigger_event(trigger_label(current_mode_), trigger_delay_ms(current_mode_));
    }

    // A mid/late-stance slip still waiting for its deadline must not fire once
    // the stance is over: toe-off (foot in the air), a new HS or an FSM resync
    // cancel it as a missed trial.
    if (state_ == State::DelayingBeforeSlip && current_mode_ != Mode::AfterHS) {
        const char* ended = nullptr;
        bus_.for_each_gait_event_since(
            cfg_.foot, last_seen_detection_count_, [&](const GaitPhase& ev) {
                if (ended) return;
                last_seen_detection_count_ = ev.detection_count;
                if (ev.phase == "TO" || ev.phase == "HS" || ev.phase == "RESET") {
                    ended = (ev.phase == "TO") ? "toe-off" : (ev.phase == "HS") ? "heel strike" : "gait FSM resync";
                }
            });
        if (ended) {
            std::cerr << "[slip] " << mode_name(current_mode_) << " CANCELLED: " << ended
                      << " before the scheduled fire -- missed trial, press '"
                      << mode_key(current_mode_) << "' again\n";
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
                est_last_hs_ns_ = ev.timestamp_ns;
                est_pred_at_hs_ns_ = estimator_.predict_stance_ns();
            } else if (ev.phase == "TO") {
                note = estimator_.on_toe_off(ev.timestamp_ns);
                if (diag_pending_ && ev.timestamp_ns > diag_hs_ns_) {
                    diag_pending_ = false;
                    const int64_t measured_ms = (ev.timestamp_ns - diag_hs_ns_) / 1000000;
                    std::cout << "[slip] estimator diagnostic: predicted stance "
                              << diag_pred_ns_ / 1000000 << " ms, measured " << measured_ms
                              << " ms (error " << (diag_pred_ns_ / 1000000 - measured_ms)
                              << " ms, slip stride)\n";
                    std::cout.flush();
                }
            } else if (ev.phase == "RESET") {
                reset_estimator("gait FSM resync");
            }
            if (!note.empty()) {
                std::cout << "[slip] stance estimator: " << note << '\n';
                std::cout.flush();
            }
        });
}

void SlipPerturbationNode::start_slip_now() {
    const int32_t velocity = slip_velocity_for(current_mode_);
    std::cout << "[slip] START foot=" << cfg_.foot << " mode=" << mode_name(current_mode_)
              << " velocity=" << velocity << " duration_ms=" << cfg_.slip_duration_ms << '\n';
    // Estimator diagnostic: where the prediction would have put this stride's
    // toe-off, and (at the next TO) how long the stance really was.
    if (est_last_hs_ns_ > 0 && est_pred_at_hs_ns_ > 0) {
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count();
        std::cout << "[slip] estimator diagnostic: fired at HS+" << (now_ns - est_last_hs_ns_) / 1000000
                  << " ms, predicted stance " << est_pred_at_hs_ns_ / 1000000 << " ms ("
                  << estimator_.describe() << ")\n";
        diag_pending_ = true;
        diag_hs_ns_ = est_last_hs_ns_;
        diag_pred_ns_ = est_pred_at_hs_ns_;
    }
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
