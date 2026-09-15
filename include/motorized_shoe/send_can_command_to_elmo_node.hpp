#ifndef MOTORIZED_SHOE_SEND_CAN_COMMAND_TO_ELMO_NODE_HPP
#define MOTORIZED_SHOE_SEND_CAN_COMMAND_TO_ELMO_NODE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "motorized_shoe/can_utils.hpp"
#include "motorized_shoe/canopen_utils.hpp"
#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"

namespace motorized_shoe {

// ELMO command node.
//
// Threading model (changed 2026-09-12):
//   * tick() runs on the 1 kHz control thread and NEVER blocks: it only sends
//     single target-velocity / controlword SDO frames and bookkeeping.
//   * Every multi-step drive procedure (initial bring-up, fault recovery,
//     re-enable after an emergency stop) runs on one dedicated worker thread,
//     one job at a time. Previously the fault-recovery re-init ran inline in
//     tick() and stalled the loop for 0.6 s per drive (1.2 s when both
//     faulted, see the 2026-09-10 logs) -- long enough to overflow the IMU
//     socket, stop the SYNC stream and miss the slip-end deadline -- and it
//     raced the still-running init thread on the same CAN socket.
//   * The worker is the only thread that READS the command socket, so SDO
//     responses are never consumed by the wrong thread. tick() must not send
//     an SDO to a node the worker is currently talking to; the ready/fault/
//     disabled gates below guarantee that.
class SendCanCommandToElmoNode {
public:
    // slip_profile_acceleration > 0 overrides the configured profile accel /
    // decel (the native SD ramp) at init; passing it here removes the old
    // "call set_slip_profile_acceleration() quickly after construction"
    // ordering hazard.
    SendCanCommandToElmoNode(const Config& cfg, DataBus& bus, int32_t slip_profile_acceleration = 0);
    ~SendCanCommandToElmoNode();

    SendCanCommandToElmoNode(const SendCanCommandToElmoNode&) = delete;
    SendCanCommandToElmoNode& operator=(const SendCanCommandToElmoNode&) = delete;

    void tick();

    // External velocity injection. While external control is active for a
    // foot, normal gait-mapped velocity commands are suppressed. inject_velocity
    // immediately sends the given velocity and marks the foot as externally
    // controlled. release_external_control restores normal gait-mapped behavior
    // and immediately sends the current gait phase's mapped velocity (or 0 in
    // slip mode, where the gait map is suppressed).
    void inject_velocity(const std::string& foot, int32_t velocity);
    void release_external_control(const std::string& foot);
    bool is_externally_controlled(const std::string& foot) const;

    // True when the foot's drive is initialized, enabled, not faulted and no
    // worker job is in flight for it -- i.e. inject_velocity() would be sent.
    // `reason` (optional) receives a short explanation when false.
    bool is_drive_available(const std::string& foot, std::string* reason = nullptr) const;
    bool is_faulted(const std::string& foot) const;

    // Emergency stop: when requested, the next tick() will inject velocity 0
    // on both feet and hold them under external control until released. The
    // atomic is safe to set from any thread (e.g. the keyboard handler); the
    // actual CAN writes still happen on the main loop thread.
    void request_emergency_stop(bool stopped);
    bool is_emergency_stopped() const;

    // Slip-experiment mode: suppress all gait-phase velocity commands so the
    // motors stay at 0 unless explicitly driven by inject_velocity (i.e. the
    // slip burst). When enabled, each foot is parked at 0 once its drive
    // finishes initializing, and again after any re-enable.
    void set_suppress_gait_velocity_commands(bool suppress);

    // Override profile acceleration/deceleration for slip-mode bursts. Prefer
    // the constructor argument; this setter only takes effect if it runs
    // before the worker reaches the accel step of the initial bring-up.
    void set_slip_profile_acceleration(int32_t accel);

    // Leave the drives safe at program exit: target velocity 0 then the
    // Shutdown controlword (motor off) on every initialized drive. Waits for
    // any in-flight worker job first. Call from the control thread after the
    // loop ends.
    void stop_all_drives();

private:
    enum class JobType { Init = 1, Recover = 2, Reenable = 3 };
    struct Job {
        JobType type;
        int node_id;
        std::string foot;
    };

    // Control-thread-only state per foot.
    struct FootState {
        std::string name;
        int node_id = 0;
        bool disabled = false;            // emergency-stopped (Shutdown sent)
        bool reenable_requested = false;  // 'r' pressed while disabled
        bool external_active = false;
        bool initial_park_done = false;
        bool fault_active = false;        // per statusword / EMCY
        int recovery_attempts = 0;
        bool gave_up = false;
        std::chrono::steady_clock::time_point last_recovery_done{};
        bool recovery_done_valid = false;
        // Bus timestamp (now_ns) when the last Recover job finished. A
        // statusword published before that instant belongs to the recovery
        // sequence itself (the drive passes through non-fault states such
        // as Switch On Disabled) and must not close the fault episode.
        int64_t recovery_done_ns = 0;
        uint32_t last_detection_count = 0;
        // Stall guard timing (see check_stall).
        bool stall_timing = false;
        std::chrono::steady_clock::time_point stall_since{};
    };

    // State shared with the worker (atomics only).
    struct FootShared {
        std::atomic<bool> ready{false};        // drive reached Operation Enabled at least once
        std::atomic<bool> job_pending{false};  // a job is queued or running for this foot
        // Completed-job notification for tick(): +type on success, -type on
        // failure, 0 = nothing new. tick() exchanges it back to 0.
        std::atomic<int> job_result{0};
    };

    FootState& state_for(const std::string& foot);
    const FootState& state_for(const std::string& foot) const;
    FootShared& shared_for(const std::string& foot);
    const FootShared& shared_for(const std::string& foot) const;

    void enqueue_job(JobType type, const FootState& foot);
    void worker_loop();
    void run_job(const Job& job);

    // Blocking drive procedures -- WORKER THREAD ONLY.
    // Full bring-up: (PDO mapping) -> NMT start -> fault reset -> shutdown ->
    // mode PV -> accel/decel/SD -> target 0 -> switch on -> enable.
    void initialize_elmo_driver(int node_id, bool configure_pdos);
    // Fast fault recovery: fault reset -> shutdown -> switch on -> target 0 ->
    // enable, each SDO confirmed, ~150 ms total. Mode/ramp survive a fault so
    // they are not rewritten. Returns true if the statusword read back
    // afterwards shows Operation Enabled without the fault bit.
    bool recover_from_fault(int node_id);
    bool write_sdo_confirmed(int node_id, uint16_t index, uint8_t subindex, uint32_t value,
                             int length, const char* what);
    bool read_sdo_u32(int node_id, uint16_t index, uint8_t subindex, uint32_t& out,
                      const char* what);
    bool run_elmo_os_command(int node_id, const std::string& command,
                             std::string* reply = nullptr);
    void configure_pdo_mapping(int node_id);

    // Single-frame sends, safe from the control thread.
    void send_velocity_command(int node_id, int32_t velocity);
    void send_controlword(int node_id, uint16_t value);
    void publish_command(const std::string& foot, int32_t velocity, uint8_t type);

    void process_foot(const GaitPhase& gait, FootState& st);
    void handle_fault(FootState& st, const ElmoStatus& status);
    // Stall guard: disable a drive that holds high current with no motion
    // and no velocity demand for longer than stall_ms_ (config).
    void check_stall(FootState& st, const ElmoMotorInfo& motor, int64_t now_ns_value);
    void disable_drive(FootState& st);
    // Worker only: read the Elmo-native failure reason (MF) and last error
    // code (EC) over the 0x1023 OS-command interface and log them decoded.
    void log_elmo_failure_reason(int node_id);

    DataBus& bus_;
    std::unique_ptr<CANSocket> can_socket_;

    FootState left_;
    FootState right_;
    FootShared left_shared_;
    FootShared right_shared_;

    // Base ELMO profile accel/decel from config, applied at init unless
    // overridden by the slip fast-ramp (slip_profile_acceleration_). Written
    // to DS402 0x6083/0x6084 and native AC/DC for consistency, and -- the
    // write that actually changes the ramp -- to the native SD param via the
    // 0x1023 OS command (see canopen_utils.hpp).
    int32_t profile_acceleration_;
    int32_t profile_deceleration_;
    std::unordered_map<std::string, int32_t> velocity_map_;
    int fault_retry_ms_;
    int fault_max_retries_;
    int stall_current_permille_;
    int stall_velocity_counts_;
    int stall_ms_;

    std::atomic<bool> emergency_stop_requested_{false};
    bool emergency_stop_applied_ = false;
    std::atomic<bool> suppress_gait_velocity_commands_{false};
    std::atomic<int32_t> slip_profile_acceleration_{0};

    // Worker thread + job queue.
    std::thread worker_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<Job> queue_;
    std::atomic<bool> shutting_down_{false};
    std::atomic<bool> worker_busy_{false};
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_SEND_CAN_COMMAND_TO_ELMO_NODE_HPP
