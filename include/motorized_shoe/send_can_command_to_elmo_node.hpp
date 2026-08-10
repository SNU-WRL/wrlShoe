#ifndef MOTORIZED_SHOE_SEND_CAN_COMMAND_TO_ELMO_NODE_HPP
#define MOTORIZED_SHOE_SEND_CAN_COMMAND_TO_ELMO_NODE_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "motorized_shoe/can_utils.hpp"
#include "motorized_shoe/canopen_utils.hpp"
#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"

namespace motorized_shoe {

class SendCanCommandToElmoNode {
public:
    SendCanCommandToElmoNode(const Config& cfg, DataBus& bus);
    ~SendCanCommandToElmoNode();

    SendCanCommandToElmoNode(const SendCanCommandToElmoNode&) = delete;
    SendCanCommandToElmoNode& operator=(const SendCanCommandToElmoNode&) = delete;

    void tick();

    // External velocity injection. While external control is active for a
    // foot, normal gait-mapped velocity commands are suppressed. inject_velocity
    // immediately sends the given velocity and marks the foot as externally
    // controlled. release_external_control restores normal gait-mapped behavior
    // and immediately sends the current gait phase's mapped velocity.
    void inject_velocity(const std::string& foot, int32_t velocity);
    void release_external_control(const std::string& foot);
    bool is_externally_controlled(const std::string& foot) const;

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

    // Override profile acceleration/deceleration for slip-mode bursts. The
    // value is written along with the park-at-0 step so that subsequent
    // inject_velocity calls hit the drive with the fast ramp already in
    // place. 0 = leave whatever init set.
    void set_slip_profile_acceleration(int32_t accel);

private:
    // configure_pdos: map the feedback TPDOs (only needed on first bring-up;
    // the mapping persists across fault-reset and NMT state changes, so the
    // fault-recovery / re-enable paths pass false to avoid re-doing the slow
    // ~260 ms PDO config and lengthening the control-loop stall).
    void initialize_elmo_driver(int node_id, bool configure_pdos = true);
    // Send an SDO download and read back the drive's response on 0x580+node_id,
    // matching the object index/subindex. Logs (and retries once) on an SDO
    // abort or missing response so a silently-dropped write -- e.g. profile
    // acceleration not taking effect -- is visible instead of mysterious.
    // Returns true only if the drive confirmed the write (0x60 response).
    bool write_sdo_confirmed(int node_id, uint16_t index, uint8_t subindex, uint32_t value,
                             int length, const char* what);
    // Expedited SDO upload (read) of a <=32-bit object into `out`. Returns true
    // only if the drive replied with a valid upload response (not an abort /
    // timeout). Used at init to read back what the drive actually stored, so a
    // write that was "accepted" but clamped/ignored is still caught.
    bool read_sdo_u32(int node_id, uint16_t index, uint8_t subindex, uint32_t& out,
                      const char* what);
    // Elmo native-interpreter command via OS-command object 0x1023 (see
    // canopen_utils.hpp). Retries once, since a concurrently-arriving SDO
    // request (e.g. the statusword poll) can abort a segmented 0x1023 transfer
    // on the drive side. Logs on failure; fills `reply` on success.
    bool run_elmo_os_command(int node_id, const std::string& command,
                             std::string* reply = nullptr);
    void configure_pdo_mapping(int node_id);
    void send_velocity_command(int node_id, int32_t velocity);
    void stop_and_reset_elmo(int node_id, const std::string& foot);
    void process_foot(const GaitPhase& gait, int node_id, const std::string& foot, uint32_t& last_detection_count);
    void disable_drive(int node_id, const std::string& foot);
    void reenable_drive(int node_id, const std::string& foot);

    DataBus& bus_;
    std::unique_ptr<CANSocket> can_socket_;

    int left_node_id_;
    int right_node_id_;
    // Base ELMO profile accel/decel from config, applied at init unless
    // overridden by the slip fast-ramp (slip_profile_acceleration_). Written
    // to DS402 0x6083/0x6084 and native AC/DC for consistency, and -- the
    // write that actually changes the ramp -- to the native SD param via the
    // 0x1023 OS command (see canopen_utils.hpp).
    int32_t profile_acceleration_;
    int32_t profile_deceleration_;
    std::unordered_map<std::string, int32_t> velocity_map_;
    std::unordered_set<std::string> active_fault_foot_names_;
    uint32_t last_left_detection_count_ = 0;
    uint32_t last_right_detection_count_ = 0;

    std::atomic<bool> left_ready_{false};
    std::atomic<bool> right_ready_{false};
    std::atomic<bool> shutting_down_{false};
    std::thread init_thread_;

    bool left_external_active_ = false;
    bool right_external_active_ = false;

    std::atomic<bool> emergency_stop_requested_{false};
    bool emergency_stop_applied_ = false;
    bool left_disabled_ = false;
    bool right_disabled_ = false;

    std::atomic<bool> suppress_gait_velocity_commands_{false};
    bool left_initial_park_done_ = false;
    bool right_initial_park_done_ = false;
    std::atomic<int32_t> slip_profile_acceleration_{0};
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_SEND_CAN_COMMAND_TO_ELMO_NODE_HPP
