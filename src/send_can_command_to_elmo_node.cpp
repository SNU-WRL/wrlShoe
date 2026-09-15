#include "motorized_shoe/send_can_command_to_elmo_node.hpp"

#include <chrono>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <thread>

namespace motorized_shoe {

namespace {
const char* job_name(int type) {
    switch (type) {
        case 1: return "init";
        case 2: return "fault-recovery";
        case 3: return "re-enable";
        default: return "?";
    }
}
}  // namespace

SendCanCommandToElmoNode::SendCanCommandToElmoNode(const Config& cfg, DataBus& bus,
                                                   int32_t slip_profile_acceleration)
    : bus_(bus),
      can_socket_(std::make_unique<CANSocket>(cfg.can_elmo_interface)),
      profile_acceleration_(cfg.profile_acceleration),
      profile_deceleration_(cfg.profile_deceleration),
      velocity_map_(cfg.velocity_map),
      fault_retry_ms_((cfg.fault_retry_ms > 0) ? cfg.fault_retry_ms : 1000),
      fault_max_retries_((cfg.fault_max_retries >= 0) ? cfg.fault_max_retries : 5),
      stall_current_permille_(cfg.stall_current_permille),
      stall_velocity_counts_(cfg.stall_velocity_counts),
      stall_ms_(cfg.stall_ms) {
    left_.name = "Left";
    left_.node_id = cfg.elmo_node_left;
    right_.name = "Right";
    right_.node_id = cfg.elmo_node_right;
    slip_profile_acceleration_.store(slip_profile_acceleration, std::memory_order_release);

    // This socket is written from the control thread (target velocity,
    // controlword) and read only by the worker during SDO exchanges. Without a
    // filter it would also collect the TPDO feedback stream it never looks at.
    can_socket_->set_filters({{0x580, 0x780}});

    // The blocking ELMO bring-up runs on the worker so the control loop and
    // the IMU drain start immediately; per-foot ready flags gate command
    // sends until each drive lands in Operation Enabled.
    worker_ = std::thread([this]() { worker_loop(); });
    enqueue_job(JobType::Init, left_);
    enqueue_job(JobType::Init, right_);
}

SendCanCommandToElmoNode::~SendCanCommandToElmoNode() {
    shutting_down_.store(true, std::memory_order_release);
    queue_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

// ---------------------------------------------------------------------------
// Small accessors

SendCanCommandToElmoNode::FootState& SendCanCommandToElmoNode::state_for(const std::string& foot) {
    return (foot == "Left") ? left_ : right_;
}
const SendCanCommandToElmoNode::FootState& SendCanCommandToElmoNode::state_for(
    const std::string& foot) const {
    return (foot == "Left") ? left_ : right_;
}
SendCanCommandToElmoNode::FootShared& SendCanCommandToElmoNode::shared_for(const std::string& foot) {
    return (foot == "Left") ? left_shared_ : right_shared_;
}
const SendCanCommandToElmoNode::FootShared& SendCanCommandToElmoNode::shared_for(
    const std::string& foot) const {
    return (foot == "Left") ? left_shared_ : right_shared_;
}

bool SendCanCommandToElmoNode::is_externally_controlled(const std::string& foot) const {
    return state_for(foot).external_active;
}

bool SendCanCommandToElmoNode::is_faulted(const std::string& foot) const {
    return state_for(foot).fault_active;
}

bool SendCanCommandToElmoNode::is_drive_available(const std::string& foot, std::string* reason) const {
    const FootState& st = state_for(foot);
    const FootShared& sh = shared_for(foot);
    const char* why = nullptr;
    if (!sh.ready.load(std::memory_order_acquire)) {
        why = "drive not initialized yet";
    } else if (st.disabled) {
        why = "drive disabled (emergency stop or stall guard; press 's' then 'r')";
    } else if (st.fault_active) {
        why = st.gave_up ? "drive fault, recovery gave up (press 's' then 'r', or restart)"
                         : "drive fault, recovery in progress";
    } else if (sh.job_pending.load(std::memory_order_acquire)) {
        why = "drive busy (init/recovery running)";
    }
    if (reason != nullptr) {
        *reason = why ? why : "";
    }
    return why == nullptr;
}

void SendCanCommandToElmoNode::request_emergency_stop(bool stopped) {
    emergency_stop_requested_.store(stopped, std::memory_order_release);
}

bool SendCanCommandToElmoNode::is_emergency_stopped() const {
    return emergency_stop_requested_.load(std::memory_order_acquire);
}

void SendCanCommandToElmoNode::set_suppress_gait_velocity_commands(bool suppress) {
    suppress_gait_velocity_commands_.store(suppress, std::memory_order_release);
}

void SendCanCommandToElmoNode::set_slip_profile_acceleration(int32_t accel) {
    slip_profile_acceleration_.store(accel, std::memory_order_release);
}

void SendCanCommandToElmoNode::publish_command(const std::string& foot, int32_t velocity, uint8_t type) {
    ElmoCommand cmd;
    cmd.timestamp_ns = now_ns();
    cmd.foot = foot;
    cmd.target_velocity = velocity;
    cmd.command_type = type;
    cmd.valid = true;
    bus_.update_command(cmd);
}

// ---------------------------------------------------------------------------
// Control-thread tick

void SendCanCommandToElmoNode::tick() {
    const auto now = std::chrono::steady_clock::now();

    // 1. Consume worker completions.
    for (FootState* st : {&left_, &right_}) {
        FootShared& sh = shared_for(st->name);
        const int res = sh.job_result.exchange(0, std::memory_order_acq_rel);
        if (res == 0) {
            continue;
        }
        const bool ok = res > 0;
        const int type = ok ? res : -res;
        if (type == static_cast<int>(JobType::Reenable)) {
            if (ok) {
                st->disabled = false;
                st->initial_park_done = false;
                publish_command(st->name, 0, 5);  // 5 = drive re-enabled
                std::cout << "[send_can_command_to_elmo] " << st->name << " re-enabled\n";
            } else {
                std::cerr << "[send_can_command_to_elmo] " << st->name
                          << " re-enable failed; press 'r' again to retry\n";
            }
        } else if (type == static_cast<int>(JobType::Recover)) {
            st->last_recovery_done = now;
            st->recovery_done_valid = true;
            st->recovery_done_ns = now_ns();
            st->initial_park_done = false;
            std::cerr << "[send_can_command_to_elmo] " << st->name << " fault-recovery attempt "
                      << st->recovery_attempts << (ok ? " OK (drive re-armed)" : " FAILED") << '\n';
        }
        // Init completion is signalled through the ready flag.
    }

    const bool left_ready = left_shared_.ready.load(std::memory_order_acquire);
    const bool right_ready = right_shared_.ready.load(std::memory_order_acquire);
    if (!left_ready && !right_ready) {
        return;
    }

    // 2. Emergency stop / resume.
    const bool stop_requested = emergency_stop_requested_.load(std::memory_order_acquire);
    if (stop_requested != emergency_stop_applied_) {
        if (stop_requested) {
            std::cout << "[motors] DISABLING drives (Shutdown control word)\n";
            std::cout.flush();
            if (left_ready && !left_.disabled) disable_drive(left_);
            if (right_ready && !right_.disabled) disable_drive(right_);
        } else {
            std::cout << "[motors] RE-ENABLING drives (re-running init on the worker, ~0.5 s each)\n";
            std::cout.flush();
            if (left_.disabled) left_.reenable_requested = true;
            if (right_.disabled) right_.reenable_requested = true;
        }
        emergency_stop_applied_ = stop_requested;
    }
    for (FootState* st : {&left_, &right_}) {
        FootShared& sh = shared_for(st->name);
        if (st->reenable_requested && !sh.job_pending.load(std::memory_order_acquire)) {
            st->reenable_requested = false;
            enqueue_job(JobType::Reenable, *st);
        }
    }

    // 3. Slip mode: park each drive at 0 once it is ready (and after any
    //    re-enable / recovery). The init already writes target 0 before
    //    enabling, so this is a belt-and-braces resend from the control
    //    thread, logged as command type 6.
    if (suppress_gait_velocity_commands_.load(std::memory_order_acquire)) {
        for (FootState* st : {&left_, &right_}) {
            FootShared& sh = shared_for(st->name);
            if (!sh.ready.load(std::memory_order_acquire) || st->disabled || st->fault_active ||
                st->initial_park_done || sh.job_pending.load(std::memory_order_acquire)) {
                continue;
            }
            try {
                send_velocity_command(st->node_id, 0);
                st->initial_park_done = true;
                publish_command(st->name, 0, 6);  // 6 = park at 0 (slip-mode suppression)
            } catch (const std::exception& e) {
                std::cerr << "[send_can_command_to_elmo] " << st->name << " park-at-0 failed: "
                          << e.what() << '\n';
            }
        }
    }

    // 4. Faults + gait-mapped commands.
    const SystemSnapshot s = bus_.snapshot();
    for (FootState* st : {&left_, &right_}) {
        FootShared& sh = shared_for(st->name);
        if (!sh.ready.load(std::memory_order_acquire) || st->disabled) {
            continue;
        }
        const ElmoStatus& status = (st->name == "Left") ? s.status_left : s.status_right;
        handle_fault(*st, status);
        if (st->disabled) {
            continue;
        }
        if (!st->fault_active && !sh.job_pending.load(std::memory_order_acquire)) {
            const ElmoMotorInfo& motor = (st->name == "Left") ? s.motor_left : s.motor_right;
            check_stall(*st, motor, s.timestamp_ns);
            if (st->disabled) {
                continue;
            }
        }

        const GaitPhase& gait = (st->name == "Left") ? s.gait_left : s.gait_right;
        if (gait.valid) {
            process_foot(gait, *st);
        }
    }
}

void SendCanCommandToElmoNode::handle_fault(FootState& st, const ElmoStatus& status) {
    const bool fault = status.valid && status.fault;
    FootShared& sh = shared_for(st.name);
    const auto now = std::chrono::steady_clock::now();

    if (!fault) {
        if (st.fault_active) {
            // The 500 ms statusword poll can catch the drive in a transient
            // non-fault state in the middle of the recovery sequence (e.g.
            // 0x0250 Switch On Disabled right after the fault reset) and,
            // seen 2026-09-15, close the episode while the Recover job was
            // still running: recovery_attempts went back to 0 ("attempt 0
            // FAILED") and fault_max_retries was never reached while the
            // drive kept re-faulting. Only a statusword read after the job
            // finished may close the episode.
            if (sh.job_pending.load(std::memory_order_acquire) ||
                (st.recovery_done_valid && status.timestamp_ns <= st.recovery_done_ns)) {
                return;
            }
            st.fault_active = false;
            st.recovery_attempts = 0;
            st.gave_up = false;
            st.recovery_done_valid = false;
            std::cerr << "[send_can_command_to_elmo] " << st.name << " drive OK again\n";
        }
        return;
    }

    if (!st.fault_active) {
        st.fault_active = true;
        st.recovery_attempts = 0;
        st.gave_up = false;
        st.recovery_done_valid = false;
        // The drive has already dropped its motor (MO=0); the wheel is free.
        // Any external control (a slip in progress) is void.
        st.external_active = false;
        std::cerr << "[send_can_command_to_elmo] " << st.name << " drive FAULT"
                  << " (statusword 0x" << std::hex << status.status_word
                  << ", error code 0x" << status.error_code << std::dec << ")\n";
    }

    if (st.gave_up || sh.job_pending.load(std::memory_order_acquire)) {
        return;
    }
    const bool due =
        !st.recovery_done_valid ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_recovery_done).count() >=
            fault_retry_ms_;
    if (!due) {
        return;
    }
    if (st.recovery_attempts >= fault_max_retries_) {
        st.gave_up = true;
        std::cerr << "[send_can_command_to_elmo] " << st.name << " still faulted after "
                  << st.recovery_attempts << " recovery attempts -- giving up. Check the error"
                  << " code above; press 's' then 'r' to retry, or restart the app.\n";
        return;
    }

    ++st.recovery_attempts;
    try {
        send_velocity_command(st.node_id, 0);  // best effort: clear a stale slip target
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << st.name << " pre-recovery velocity 0 failed: "
                  << e.what() << '\n';
    }
    publish_command(st.name, 0, 1);  // 1 = fault stop + recovery
    std::cerr << "[send_can_command_to_elmo] re-arming " << st.name << " drive (node " << st.node_id
              << "), attempt " << st.recovery_attempts << "/" << fault_max_retries_ << '\n';
    enqueue_job(JobType::Recover, st);
}

void SendCanCommandToElmoNode::process_foot(const GaitPhase& gait, FootState& st) {
    if (st.fault_active || st.external_active ||
        suppress_gait_velocity_commands_.load(std::memory_order_acquire) ||
        shared_for(st.name).job_pending.load(std::memory_order_acquire)) {
        st.last_detection_count = gait.detection_count;
        return;
    }

    if (gait.detection_count == st.last_detection_count) {
        return;
    }
    st.last_detection_count = gait.detection_count;

    int32_t target_velocity = 0;
    const auto it = velocity_map_.find(gait.phase);
    if (it != velocity_map_.end()) {
        target_velocity = it->second;
    }

    try {
        send_velocity_command(st.node_id, target_velocity);
    } catch (const std::exception& e) {
        // Transient CAN write failure (e.g. ENOBUFS): log and retry on the
        // next phase change. Not a drive fault.
        std::cerr << "[send_can_command_to_elmo] " << st.name
                  << " command send failed: " << e.what() << '\n';
        return;
    }
    publish_command(st.name, target_velocity, 0);
}

void SendCanCommandToElmoNode::inject_velocity(const std::string& foot, int32_t velocity) {
    std::string reason;
    if (!is_drive_available(foot, &reason)) {
        std::cerr << "[send_can_command_to_elmo] " << foot << " inject_velocity ignored: " << reason
                  << '\n';
        return;
    }
    FootState& st = state_for(foot);
    try {
        send_velocity_command(st.node_id, velocity);
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " inject_velocity failed: " << e.what() << '\n';
        return;
    }
    st.external_active = true;
    publish_command(foot, velocity, 2);  // 2 = external injection (slip perturbation)
    std::cout << "[send_can_command_to_elmo] " << foot << " inject_velocity sent: " << velocity
              << " (node " << st.node_id << ")\n";
    std::cout.flush();
}

void SendCanCommandToElmoNode::release_external_control(const std::string& foot) {
    FootState& st = state_for(foot);
    st.external_active = false;

    if (!is_drive_available(foot)) {
        return;
    }

    // In slip mode the gait map is suppressed: releasing means "hold 0", never
    // the Swing velocity (the old code could re-spin the motor here).
    int32_t target_velocity = 0;
    if (!suppress_gait_velocity_commands_.load(std::memory_order_acquire)) {
        const SystemSnapshot s = bus_.snapshot();
        const GaitPhase& gait = (foot == "Left") ? s.gait_left : s.gait_right;
        if (!gait.valid) {
            return;
        }
        const auto it = velocity_map_.find(gait.phase);
        if (it != velocity_map_.end()) {
            target_velocity = it->second;
        }
    }

    try {
        send_velocity_command(st.node_id, target_velocity);
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " release failed: " << e.what() << '\n';
        return;
    }
    publish_command(foot, target_velocity, 3);  // 3 = post-slip release back to gait map
}

void SendCanCommandToElmoNode::disable_drive(FootState& st) {
    try {
        send_velocity_command(st.node_id, 0);
        send_controlword(st.node_id, CANOPEN_SHUTDOWN_STATE);
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << st.name
                  << " disable failed: " << e.what() << '\n';
        return;
    }
    st.disabled = true;
    st.external_active = false;
    publish_command(st.name, 0, 4);  // 4 = drive disabled (Shutdown)
    std::cout << "[send_can_command_to_elmo] " << st.name << " disabled\n";
    std::cout.flush();
}

void SendCanCommandToElmoNode::check_stall(FootState& st, const ElmoMotorInfo& motor,
                                           int64_t now_ns_value) {
    if (stall_ms_ <= 0 || stall_current_permille_ <= 0) {
        return;
    }
    // Feedback must be live: the TPDOs stop when the drive is off or the bus
    // is quiet, and a stale sample must not keep the timer running.
    const bool fresh = motor.current_valid && motor.velocity_valid && motor.velocity_demand_valid &&
                       (now_ns_value - motor.timestamp_ns) < 500000000LL;  // 500 ms
    const bool stalled = fresh &&
                         std::abs(static_cast<int>(motor.current)) >= stall_current_permille_ &&
                         std::abs(motor.velocity) <= stall_velocity_counts_ &&
                         motor.velocity_demand == 0;
    const auto now = std::chrono::steady_clock::now();
    if (!stalled) {
        st.stall_timing = false;
        return;
    }
    if (!st.stall_timing) {
        st.stall_timing = true;
        st.stall_since = now;
        return;
    }
    const auto held_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.stall_since).count();
    if (held_ms < stall_ms_) {
        return;
    }
    st.stall_timing = false;
    std::cerr << "[send_can_command_to_elmo] " << st.name << " STALL GUARD: current "
              << motor.current << " permille with velocity " << motor.velocity
              << " counts/s and velocity demand 0 for " << held_ms
              << " ms -- disabling the drive (Shutdown) to protect the motor."
              << " Check the wheel; press 's' then 'r' to re-enable.\n";
    disable_drive(st);
    if (st.disabled) {
        publish_command(st.name, 0, 7);  // 7 = stall guard trip
    }
}

namespace {

// Elmo Gold "MF" (Motor Fault) bit field, from the Command Reference for
// Gold Line Drives (MAN-G-CR 1.406, MF command, pp. 192-195). The CAN EMCY
// code each bit maps to is given in brackets. Bits 12-15 are a 4-bit
// amplifier-status field, not individual flags.
std::string decode_elmo_mf(uint32_t mf) {
    if (mf == 0) {
        return "no failure recorded (MF=0; motor was shut off by a command, not a fault)";
    }
    std::string out;
    auto add = [&](const std::string& s) {
        if (!out.empty()) out += "; ";
        out += s;
    };
    struct Flag { uint32_t bit; const char* label; };
    static const Flag kFlags[] = {
        {0x1u,        "main feedback error [7300]"},
        {0x2u,        "commutation process failed during motor on"},
        {0x4u,        "Hall / main feedback mismatch [7380]"},
        {0x8u,        "current exceeded peak limit MC [8311]"},
        {0x10u,       "external inhibit INH/ENB triggered [5441]"},
        {0x40u,       "Hall sensor speed too high [7381]"},
        {0x80u,       "speed tracking error ER[2] [8480]"},
        {0x100u,      "position tracking error ER[3] [8611]"},
        {0x800u,      "heartbeat event [8130]"},
        {0x20000u,    "overspeed HL[2]/LL[2] [8481]"},
        {0x200000u,   "motor stuck CL[2..4] [7121]"},
        {0x400000u,   "feedback out of position limits HL[3]/LL[3] [8680]"},
        {0x800000u,   "numeric overflow [FF30]"},
        {0x1000000u,  "gantry slave disabled"},
        {0x20000000u, "failed to start motor [FF10]: inhibit active, commutation auto-phasing "
                      "failed, < 7.5 ms since last fault/disable, profiler conflict (EE[2]), "
                      "or motor rotating too fast at enable (EC 168)"},
    };
    uint32_t seen = 0;
    for (const Flag& f : kFlags) {
        if (mf & f.bit) { add(f.label); seen |= f.bit; }
    }
    const uint32_t amp = mf & 0xF000u;
    if (amp != 0) {
        seen |= 0xF000u;
        switch (amp) {
            case 0x3000u: add("amplifier: under-voltage [3120] (bus voltage AN[6])"); break;
            case 0x5000u: add("amplifier: over-voltage [3310] (bus voltage AN[6])"); break;
            case 0x7000u: add("amplifier: safety input (STO) [FF20]"); break;
            case 0xB000u: add("amplifier: short protection [2340]"); break;
            case 0xD000u: add("amplifier: over-temperature [4310] (TI[1] = drive temperature)"); break;
            default: {
                std::ostringstream o; o << "amplifier status 0x" << std::hex << amp; add(o.str());
            }
        }
    }
    for (int bit = 0; bit < 32; ++bit) {
        const uint32_t b = 1u << bit;
        if ((mf & b) && !(seen & b)) {
            add("reserved/unknown MF bit " + std::to_string(bit));
        }
    }
    return out;
}

// Elmo "EC" (error code of the last failed command), same manual p. 99. Only
// the one we have met is named; everything else prints raw.
const char* decode_elmo_ec(long ec) {
    switch (ec) {
        case 0:   return "no error";
        case 168: return "SPEED_2_LARGE_2_START: motor was enabled while rotating too fast";
        case 169: return "CPU peripheral busy / overflow";
        default:  return nullptr;
    }
}

}  // namespace

void SendCanCommandToElmoNode::log_elmo_failure_reason(int node_id) {
    // The next motor-enable clears MF, so this runs before the fault reset.
    // TI[1] is the drive temperature in degrees C (relevant for MF 0xD000 /
    // EMCY 0x4310 and not available through any CiA-402 object).
    std::string mf_reply, ec_reply, ti_reply;
    const bool have_mf = run_elmo_os_command(node_id, "MF", &mf_reply);
    const bool have_ec = run_elmo_os_command(node_id, "EC", &ec_reply);
    const bool have_ti = run_elmo_os_command(node_id, "TI[1]", &ti_reply);
    std::cerr << "[send_can_command_to_elmo] node " << node_id << " Elmo failure reason: ";
    if (have_mf) {
        const uint32_t mf = static_cast<uint32_t>(std::strtoul(mf_reply.c_str(), nullptr, 10));
        std::cerr << "MF=0x" << std::hex << mf << std::dec << " (" << decode_elmo_mf(mf) << ")";
    } else {
        std::cerr << "MF unreadable";
    }
    if (have_ec) {
        const long ec = std::strtol(ec_reply.c_str(), nullptr, 10);
        const char* label = decode_elmo_ec(ec);
        std::cerr << "; EC=" << ec << " (" << (label ? label : "see Elmo Command Reference, EC") << ")";
    } else {
        std::cerr << "; EC unreadable";
    }
    if (have_ti) {
        std::cerr << "; drive temperature TI[1]=" << ti_reply << " C";
    }
    std::cerr << '\n';
}

void SendCanCommandToElmoNode::stop_all_drives() {
    // Let an in-flight worker job finish (bounded) so we do not interleave
    // with its SDO exchange, then stop taking new jobs.
    for (int i = 0; i < 100 && worker_busy_.load(std::memory_order_acquire); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    shutting_down_.store(true, std::memory_order_release);
    queue_cv_.notify_all();

    for (FootState* st : {&left_, &right_}) {
        if (!shared_for(st->name).ready.load(std::memory_order_acquire)) {
            continue;
        }
        try {
            send_velocity_command(st->node_id, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            send_controlword(st->node_id, CANOPEN_SHUTDOWN_STATE);
            std::cout << "[send_can_command_to_elmo] " << st->name
                      << " stopped and disabled at exit\n";
        } catch (const std::exception& e) {
            std::cerr << "[send_can_command_to_elmo] " << st->name
                      << " stop at exit failed: " << e.what() << '\n';
        }
        st->disabled = true;
    }
    std::cout.flush();
}

// ---------------------------------------------------------------------------
// Single-frame sends

void SendCanCommandToElmoNode::send_velocity_command(int node_id, int32_t velocity) {
    auto vel = create_sdo_download(node_id, CANOPEN_TARGET_VELOCITY, 0, static_cast<uint32_t>(velocity), 4);
    can_socket_->send_message(vel.can_id, vel.data, vel.dlc);
}

void SendCanCommandToElmoNode::send_controlword(int node_id, uint16_t value) {
    auto cw = create_sdo_download(node_id, CANOPEN_CONTROL_WORD, 0, value, 2);
    can_socket_->send_message(cw.can_id, cw.data, cw.dlc);
}

// ---------------------------------------------------------------------------
// Worker

void SendCanCommandToElmoNode::enqueue_job(JobType type, const FootState& foot) {
    shared_for(foot.name).job_pending.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push_back(Job{type, foot.node_id, foot.name});
    }
    queue_cv_.notify_one();
}

void SendCanCommandToElmoNode::worker_loop() {
    while (true) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [&] {
                return shutting_down_.load(std::memory_order_acquire) || !queue_.empty();
            });
            if (shutting_down_.load(std::memory_order_acquire)) {
                // Drop queued jobs; mark them not pending so nothing waits.
                for (const auto& j : queue_) {
                    shared_for(j.foot).job_pending.store(false, std::memory_order_release);
                }
                queue_.clear();
                return;
            }
            job = queue_.front();
            queue_.pop_front();
        }
        worker_busy_.store(true, std::memory_order_release);
        run_job(job);
        worker_busy_.store(false, std::memory_order_release);
    }
}

void SendCanCommandToElmoNode::run_job(const Job& job) {
    FootShared& sh = shared_for(job.foot);
    bool ok = false;
    try {
        switch (job.type) {
            case JobType::Init:
                initialize_elmo_driver(job.node_id, /*configure_pdos=*/true);
                ok = true;
                break;
            case JobType::Reenable:
                initialize_elmo_driver(job.node_id, /*configure_pdos=*/false);
                ok = true;
                break;
            case JobType::Recover:
                ok = recover_from_fault(job.node_id);
                break;
        }
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << job.foot << " " << job_name(static_cast<int>(job.type))
                  << " failed: " << e.what() << '\n';
        ok = false;
    }
    if (ok && (job.type == JobType::Init || job.type == JobType::Reenable)) {
        sh.ready.store(true, std::memory_order_release);
    }
    const int code = static_cast<int>(job.type);
    sh.job_result.store(ok ? code : -code, std::memory_order_release);
    sh.job_pending.store(false, std::memory_order_release);
}

bool SendCanCommandToElmoNode::write_sdo_confirmed(int node_id, uint16_t index, uint8_t subindex,
                                                   uint32_t value, int length, const char* what) {
    using namespace std::chrono_literals;
    const uint32_t expect_id = encode_canopen_sdo_rx_id(static_cast<uint32_t>(node_id));

    // Two attempts: the drive has been observed to silently abort a write under
    // tight SDO timing, and a single retry after a short settle usually lands it.
    for (int attempt = 1; attempt <= 2; ++attempt) {
        auto msg = create_sdo_download(node_id, index, subindex, value, length);
        can_socket_->send_message(msg.can_id, msg.data, msg.dlc);

        const auto deadline = std::chrono::steady_clock::now() + 100ms;
        bool got_response = false;
        bool accepted = false;
        uint32_t abort_code = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto rem = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 deadline - std::chrono::steady_clock::now())
                                 .count();
            if (rem <= 0) {
                break;
            }
            uint32_t rx_id = 0;
            uint8_t d[8] = {0};
            size_t len = 0;
            bool ok = false;
            try {
                ok = can_socket_->recv_message(rx_id, d, len, static_cast<int>(rem));
            } catch (const std::exception& e) {
                std::cerr << "[send_can_command_to_elmo] node " << node_id
                          << " SDO confirm recv error: " << e.what() << '\n';
                break;
            }
            if (!ok) {
                break;  // poll timeout: no frame arrived
            }
            // Only the matching node's SDO server response for THIS object counts;
            // skip other nodes' responses and responses for other objects.
            if (rx_id != expect_id || len < 4) {
                continue;
            }
            const uint16_t resp_index = static_cast<uint16_t>(d[1] | (d[2] << 8));
            if (resp_index != index || d[3] != subindex) {
                continue;
            }
            got_response = true;
            if (d[0] == 0x60) {
                accepted = true;
            } else if (d[0] == 0x80) {
                abort_code = static_cast<uint32_t>(d[4]) |
                             (static_cast<uint32_t>(d[5]) << 8) |
                             (static_cast<uint32_t>(d[6]) << 16) |
                             (static_cast<uint32_t>(d[7]) << 24);
            }
            break;
        }

        if (accepted) {
            if (attempt > 1) {
                std::cerr << "[send_can_command_to_elmo] node " << node_id << " SDO " << what
                          << " confirmed on retry " << attempt << '\n';
            }
            return true;
        }

        const bool last = (attempt == 2);
        if (got_response) {
            std::cerr << "[send_can_command_to_elmo] node " << node_id << " SDO " << what
                      << " (0x" << std::hex << index << ":" << std::dec
                      << static_cast<int>(subindex) << " = " << value << ") ABORTED, code 0x"
                      << std::hex << abort_code << std::dec << (last ? "" : "; retrying") << '\n';
        } else {
            std::cerr << "[send_can_command_to_elmo] node " << node_id << " SDO " << what
                      << " (0x" << std::hex << index << std::dec << " = " << value
                      << ") got NO response" << (last ? "" : "; retrying") << '\n';
        }
        if (!last) {
            std::this_thread::sleep_for(20ms);
        }
    }

    std::cerr << "[send_can_command_to_elmo] node " << node_id << " SDO " << what
              << " NOT confirmed after retries -- drive is likely using its stored value\n";
    return false;
}

bool SendCanCommandToElmoNode::read_sdo_u32(int node_id, uint16_t index, uint8_t subindex,
                                            uint32_t& out, const char* what) {
    using namespace std::chrono_literals;
    const uint32_t expect_id = encode_canopen_sdo_rx_id(static_cast<uint32_t>(node_id));

    auto req = create_sdo_upload(static_cast<uint32_t>(node_id), index, subindex);
    can_socket_->send_message(req.can_id, req.data, req.dlc);

    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto rem = std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - std::chrono::steady_clock::now())
                             .count();
        if (rem <= 0) {
            break;
        }
        uint32_t rx_id = 0;
        uint8_t d[8] = {0};
        size_t len = 0;
        bool ok = false;
        try {
            ok = can_socket_->recv_message(rx_id, d, len, static_cast<int>(rem));
        } catch (const std::exception& e) {
            std::cerr << "[send_can_command_to_elmo] node " << node_id << " SDO read " << what
                      << " recv error: " << e.what() << '\n';
            break;
        }
        if (!ok) {
            break;
        }
        if (rx_id != expect_id || len < 4) {
            continue;
        }
        const uint16_t resp_index = static_cast<uint16_t>(d[1] | (d[2] << 8));
        if (resp_index != index || d[3] != subindex) {
            continue;
        }
        if (d[0] == 0x80) {
            const uint32_t abort_code = static_cast<uint32_t>(d[4]) |
                                        (static_cast<uint32_t>(d[5]) << 8) |
                                        (static_cast<uint32_t>(d[6]) << 16) |
                                        (static_cast<uint32_t>(d[7]) << 24);
            std::cerr << "[send_can_command_to_elmo] node " << node_id << " SDO read " << what
                      << " ABORTED, code 0x" << std::hex << abort_code << std::dec << '\n';
            return false;
        }
        if ((d[0] & 0xE0) == 0x40) {  // upload (read) response
            out = static_cast<uint32_t>(d[4]) | (static_cast<uint32_t>(d[5]) << 8) |
                  (static_cast<uint32_t>(d[6]) << 16) | (static_cast<uint32_t>(d[7]) << 24);
            return true;
        }
        return false;
    }
    std::cerr << "[send_can_command_to_elmo] node " << node_id << " SDO read " << what
              << " got NO response within timeout\n";
    return false;
}

bool SendCanCommandToElmoNode::run_elmo_os_command(int node_id, const std::string& command,
                                                   std::string* reply) {
    using namespace std::chrono_literals;
    for (int attempt = 1; attempt <= 2; ++attempt) {
        const ElmoOsCommandResult res = elmo_os_command(*can_socket_, node_id, command);
        if (res.ok) {
            if (reply != nullptr) {
                *reply = res.reply;
            }
            if (attempt > 1) {
                std::cerr << "[send_can_command_to_elmo] node " << node_id << " OS command '"
                          << command << "' succeeded on retry " << attempt << '\n';
            }
            return true;
        }
        const bool last = (attempt == 2);
        std::cerr << "[send_can_command_to_elmo] node " << node_id << " OS command '" << command
                  << "' failed: " << res.error << (last ? "" : "; retrying") << '\n';
        if (!last) {
            std::this_thread::sleep_for(20ms);
        }
    }
    return false;
}

void SendCanCommandToElmoNode::initialize_elmo_driver(int node_id, bool configure_pdos) {
    using namespace std::chrono_literals;

    // Stale responses from an earlier exchange must not be mistaken for the
    // replies to this one.
    can_socket_->drain();

    // PDO mapping must be edited while the node is NMT Pre-Operational. Force
    // pre-op first (a node coming out of power-up is already there, but a prior
    // run may have left it Operational), configure the TPDOs, then NMT Start so
    // the SYNC-triggered TPDOs begin transmitting.
    if (configure_pdos) {
        auto preop = create_nmt_message(node_id, CANOPEN_NMT_PREOP);
        can_socket_->send_message(preop.can_id, preop.data, preop.dlc);
        std::this_thread::sleep_for(50ms);

        configure_pdo_mapping(node_id);
    }

    auto nmt = create_nmt_message(node_id, CANOPEN_NMT_START);
    can_socket_->send_message(nmt.can_id, nmt.data, nmt.dlc);
    std::this_thread::sleep_for(100ms);

    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_FAULT_RESET, 2, "fault-reset");
    std::this_thread::sleep_for(50ms);

    // Force the motor off (Shutdown -> Ready to Switch On) BEFORE touching the
    // profiler parameters: the native AC/DC writes below are rejected while the
    // motor is on, and a hard-killed previous session can leave the drive in
    // Operation Enabled through the NMT/fault-reset preamble above.
    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SHUTDOWN_STATE, 2, "shutdown");
    std::this_thread::sleep_for(50ms);

    // Mode of operation = 3 (Profile Velocity). Confirmed: if this is not applied
    // the drive is not in pv mode and velocity commands are not profiled at all.
    write_sdo_confirmed(node_id, CANOPEN_MODE_OF_OPERATION, 0, 3, 1, "mode-of-operation=pv");
    std::this_thread::sleep_for(50ms);

    // Profile acceleration / deceleration. The base values come from config
    // (profile_acceleration_ / profile_deceleration_) and can differ.
    // Slip mode wants a fast symmetric ramp so the motor reaches the commanded
    // slip velocity before the burst ends; when slip_profile_acceleration_ > 0
    // it overrides BOTH accel and decel.
    const int32_t slip_accel = slip_profile_acceleration_.load(std::memory_order_acquire);
    const bool slip_override = slip_accel > 0;
    const uint32_t accel_value =
        slip_override ? static_cast<uint32_t>(slip_accel) : static_cast<uint32_t>(profile_acceleration_);
    const uint32_t decel_value =
        slip_override ? static_cast<uint32_t>(slip_accel) : static_cast<uint32_t>(profile_deceleration_);

    // DS402 profile accel/decel. On these drives this write is stored and reads
    // back correctly but does NOT reach the trajectory generator (the mystery
    // 1e6 ramp -- see canopen_utils.hpp). Kept so the DS402 objects stay
    // consistent with what we actually configure below.
    write_sdo_confirmed(node_id, 0x6083, 0, accel_value, 4, "profile-acceleration");
    std::this_thread::sleep_for(50ms);

    write_sdo_confirmed(node_id, 0x6084, 0, decel_value, 4, "profile-deceleration");
    std::this_thread::sleep_for(50ms);

    // Native AC/DC via the 0x1023 OS command. Like 0x6083/0x6084 these are
    // stored but ignored by the PV trajectory generator in UM=5; written only
    // to keep every accel-shaped parameter consistent.
    run_elmo_os_command(node_id, "AC=" + std::to_string(accel_value));
    std::this_thread::sleep_for(50ms);

    run_elmo_os_command(node_id, "DC=" + std::to_string(decel_value));
    std::this_thread::sleep_for(50ms);

    // The write that actually changes the ramp: native SD ("stop deceleration")
    // via the 0x1023 OS command. Measured on hardware 2026-07-16: the PV-mode
    // demand slope follows SD in BOTH directions and ignores AC/DC/0x6083/
    // 0x6084 entirely (see canopen_utils.hpp). SD is volatile (power cycle
    // restores the flash default, 1e6 on our drives), so this must run on
    // every init, while the motor is off (guaranteed by the Shutdown
    // controlword above). SD is a single magnitude: it cannot honor an
    // asymmetric accel/decel pair, so the accel value wins.
    if (accel_value != decel_value) {
        std::cerr << "[send_can_command_to_elmo] node " << node_id
                  << " WARNING: profile accel " << accel_value << " != decel " << decel_value
                  << "; the drive ramps with a single SD parameter, using accel value\n";
    }
    run_elmo_os_command(node_id, "SD=" + std::to_string(accel_value));
    std::this_thread::sleep_for(50ms);

    // Definitive readback: query the native SD the profiler actually uses.
    std::string sd_reply;
    if (run_elmo_os_command(node_id, "SD", &sd_reply)) {
        const long long rb_sd = std::strtoll(sd_reply.c_str(), nullptr, 10);
        std::cout << "[send_can_command_to_elmo] node " << node_id
                  << " native ramp (SD) readback: SD=" << sd_reply
                  << " (wrote " << accel_value << ")"
                  << ((rb_sd == static_cast<long long>(accel_value))
                          ? " OK"
                          : " MISMATCH -- drive is not using the commanded ramp")
                  << '\n';
        std::cout.flush();
    } else {
        std::cerr << "[send_can_command_to_elmo] node " << node_id
                  << " SD readback failed -- ramp NOT verified\n";
    }

    // Target velocity 0 BEFORE enabling: 0x60FF keeps its last value across
    // fault/disable, so a session killed mid-slip would otherwise spin the
    // motor at the slip velocity the instant Operation Enabled lands.
    write_sdo_confirmed(node_id, CANOPEN_TARGET_VELOCITY, 0, 0, 4, "target-velocity=0");
    std::this_thread::sleep_for(20ms);

    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SWITCH_ON_STATE, 2, "switch-on");
    std::this_thread::sleep_for(50ms);

    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_ENABLE_OPERATION_STATE, 2,
                        "enable-operation");
    std::this_thread::sleep_for(50ms);

    uint32_t sw = 0;
    if (read_sdo_u32(node_id, CANOPEN_STATUS_WORD, 0, sw, "statusword")) {
        const bool oe = (sw & STATUS_OPERATION_ENABLED) != 0;
        const bool fault = (sw & STATUS_FAULT) != 0;
        std::cout << "[send_can_command_to_elmo] node " << node_id << " init done: statusword 0x"
                  << std::hex << (sw & 0xFFFF) << std::dec
                  << (fault ? " FAULT" : (oe ? " Operation Enabled" : " NOT enabled")) << '\n';
        std::cout.flush();
    }
}

bool SendCanCommandToElmoNode::recover_from_fault(int node_id) {
    using namespace std::chrono_literals;
    can_socket_->drain();

    // Why did it trip? The EMCY code is CiA-generic (or 0xFF10 manufacturer-
    // specific, seen at init on 2026-09-15); the drive's own MF bitmask says
    // exactly what happened, and the fault reset below clears it.
    log_elmo_failure_reason(node_id);

    // Rising edge on controlword bit 7 clears the fault; then walk the CiA-402
    // state machine back up. Mode of operation and the SD ramp survive a
    // fault, so they are not rewritten here (that is what kept the old
    // recovery at ~600 ms).
    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_FAULT_RESET, 2, "fault-reset");
    std::this_thread::sleep_for(20ms);
    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SHUTDOWN_STATE, 2, "shutdown");
    std::this_thread::sleep_for(20ms);
    write_sdo_confirmed(node_id, CANOPEN_TARGET_VELOCITY, 0, 0, 4, "target-velocity=0");
    std::this_thread::sleep_for(20ms);
    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SWITCH_ON_STATE, 2, "switch-on");
    std::this_thread::sleep_for(20ms);
    write_sdo_confirmed(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_ENABLE_OPERATION_STATE, 2,
                        "enable-operation");
    std::this_thread::sleep_for(30ms);

    uint32_t sw = 0;
    if (!read_sdo_u32(node_id, CANOPEN_STATUS_WORD, 0, sw, "statusword")) {
        return false;
    }
    const bool oe = (sw & STATUS_OPERATION_ENABLED) != 0;
    const bool fault = (sw & STATUS_FAULT) != 0;
    std::cerr << "[send_can_command_to_elmo] node " << node_id << " recovery statusword 0x"
              << std::hex << (sw & 0xFFFF) << std::dec
              << (fault ? " (still FAULT)" : (oe ? " (Operation Enabled)" : " (not enabled)")) << '\n';
    return oe && !fault;
}

void SendCanCommandToElmoNode::configure_pdo_mapping(int node_id) {
    using namespace std::chrono_literals;

    // Configure two SYNC-triggered TPDOs per drive so motor feedback streams at
    // the SYNC rate (1 kHz) without per-value SDO request frames:
    //   TPDO1 (0x180+id): position 0x6064 (32b) + velocity 0x606C (32b) = 8 B
    //   TPDO2 (0x280+id): current 0x6078 (16b) + velocity demand 0x606B (32b)
    //                     + current demand 0x6074 (16b)                 = 8 B
    // Statusword stays on the SDO poll (+ EMCY) for fault detection.
    //
    // Standard remap procedure per PDO: disable the PDO (COB-ID bit 31), set the
    // transmission type, clear the mapping count, write the mapping entries, set
    // the count, then re-enable the COB-ID. Writes are blind (fire-and-forget
    // with a short gap); the inter-write gap keeps the drive's SDO server from
    // collapsing back-to-back transfers.
    const uint32_t id = static_cast<uint32_t>(node_id);
    auto write_sdo = [&](uint16_t index, uint8_t sub, uint32_t value, int length) {
        auto msg = create_sdo_download(node_id, index, sub, value, length);
        can_socket_->send_message(msg.can_id, msg.data, msg.dlc);
        std::this_thread::sleep_for(20ms);
    };

    // Mapping entry encoding: (index << 16) | (subindex << 8) | bit_length.
    const uint32_t map_position = (static_cast<uint32_t>(CANOPEN_POSITION_ACTUAL) << 16) | 0x20;
    const uint32_t map_velocity = (static_cast<uint32_t>(CANOPEN_VELOCITY_ACTUAL) << 16) | 0x20;
    const uint32_t map_current = (static_cast<uint32_t>(CANOPEN_CURRENT_ACTUAL) << 16) | 0x10;
    const uint32_t map_vel_demand = (static_cast<uint32_t>(CANOPEN_VELOCITY_DEMAND) << 16) | 0x20;
    const uint32_t map_current_demand = (static_cast<uint32_t>(CANOPEN_CURRENT_DEMAND) << 16) | 0x10;

    const uint32_t cob1 = CANOPEN_TPDO1_COB_BASE + id;
    write_sdo(CANOPEN_TPDO1_COMM, 1, cob1 | 0x80000000u, 4);  // disable
    write_sdo(CANOPEN_TPDO1_COMM, 2, 1, 1);                   // transmit on every SYNC
    write_sdo(CANOPEN_TPDO1_MAP, 0, 0, 1);                    // clear mapping
    write_sdo(CANOPEN_TPDO1_MAP, 1, map_position, 4);
    write_sdo(CANOPEN_TPDO1_MAP, 2, map_velocity, 4);
    write_sdo(CANOPEN_TPDO1_MAP, 0, 2, 1);                    // two entries
    write_sdo(CANOPEN_TPDO1_COMM, 1, cob1, 4);               // re-enable

    const uint32_t cob2 = CANOPEN_TPDO2_COB_BASE + id;
    write_sdo(CANOPEN_TPDO2_COMM, 1, cob2 | 0x80000000u, 4);  // disable
    write_sdo(CANOPEN_TPDO2_COMM, 2, 1, 1);                   // transmit on every SYNC
    write_sdo(CANOPEN_TPDO2_MAP, 0, 0, 1);                    // clear mapping
    write_sdo(CANOPEN_TPDO2_MAP, 1, map_current, 4);
    write_sdo(CANOPEN_TPDO2_MAP, 2, map_vel_demand, 4);
    write_sdo(CANOPEN_TPDO2_MAP, 3, map_current_demand, 4);
    write_sdo(CANOPEN_TPDO2_MAP, 0, 3, 1);                    // three entries
    write_sdo(CANOPEN_TPDO2_COMM, 1, cob2, 4);               // re-enable

    // The blind writes above leave their acks queued; clear them so the
    // confirmed writes that follow see only their own responses.
    can_socket_->drain();
}

}  // namespace motorized_shoe
