#include "motorized_shoe/send_can_command_to_elmo_node.hpp"

#include <iostream>
#include <thread>

namespace motorized_shoe {

SendCanCommandToElmoNode::SendCanCommandToElmoNode(const Config& cfg, DataBus& bus)
    : bus_(bus),
      can_socket_(std::make_unique<CANSocket>(cfg.can_elmo_interface)),
      left_node_id_(cfg.elmo_node_left),
      right_node_id_(cfg.elmo_node_right),
      profile_acceleration_(cfg.profile_acceleration),
      profile_deceleration_(cfg.profile_deceleration),
      velocity_map_(cfg.velocity_map) {
    // Run the blocking ELMO init off the main thread. The init sequence
    // issues ~7 SDO writes with 50 ms gaps on can0; running it inline blocks
    // the constructor for ~400 ms, during which the can1 IMU socket fills
    // and the shared SPI bus saturates. The control loop and IMU drain start
    // immediately; per-foot ready flags gate command sends until init lands.
    init_thread_ = std::thread([this]() {
        try {
            initialize_elmo_driver(left_node_id_);
            left_ready_.store(true, std::memory_order_release);
        } catch (const std::exception& e) {
            std::cerr << "[send_can_command_to_elmo] Left ELMO init failed: " << e.what() << '\n';
        }

        if (shutting_down_.load(std::memory_order_acquire)) {
            return;
        }

        try {
            initialize_elmo_driver(right_node_id_);
            right_ready_.store(true, std::memory_order_release);
        } catch (const std::exception& e) {
            std::cerr << "[send_can_command_to_elmo] Right ELMO init failed: " << e.what() << '\n';
        }
    });
}

SendCanCommandToElmoNode::~SendCanCommandToElmoNode() {
    shutting_down_.store(true, std::memory_order_release);
    if (init_thread_.joinable()) {
        init_thread_.join();
    }
}

void SendCanCommandToElmoNode::tick() {
    const bool left_ready = left_ready_.load(std::memory_order_acquire);
    const bool right_ready = right_ready_.load(std::memory_order_acquire);

    if (!left_ready && !right_ready) {
        return;
    }

    const bool stop_requested = emergency_stop_requested_.load(std::memory_order_acquire);
    if (stop_requested != emergency_stop_applied_) {
        if (stop_requested) {
            std::cout << "[motors] DISABLING drives (Shutdown control word)\n";
            std::cout.flush();
            if (left_ready && !left_disabled_) {
                disable_drive(left_node_id_, "Left");
            }
            if (right_ready && !right_disabled_) {
                disable_drive(right_node_id_, "Right");
            }
        } else {
            std::cout << "[motors] RE-ENABLING drives (re-running init, ~0.5 s each)\n";
            std::cout.flush();
            if (left_disabled_) {
                reenable_drive(left_node_id_, "Left");
            }
            if (right_disabled_) {
                reenable_drive(right_node_id_, "Right");
            }
        }
        emergency_stop_applied_ = stop_requested;
    }

    // In slip-experiment mode, explicitly send velocity 0 to each drive the
    // first time it becomes ready (and again after any re-enable). The init
    // sequence ends in Operation Enabled but does not write target_velocity,
    // so the drive could otherwise hold whatever stale value it had.
    if (suppress_gait_velocity_commands_.load(std::memory_order_acquire)) {
        // Park each drive at 0 once it finishes init (and again after a
        // re-enable). Only the target-velocity SDO is written here — the
        // profile acceleration override is applied by the init thread with
        // proper 50 ms spacing between SDOs; back-to-back SDO writes from
        // the real-time tick can land in the drive's SDO server within the
        // same processing window and silently abort.
        if (left_ready && !left_disabled_ && !left_initial_park_done_) {
            try {
                send_velocity_command(left_node_id_, 0);
                left_initial_park_done_ = true;
                ElmoCommand cmd;
                cmd.timestamp_ns = now_ns();
                cmd.foot = "Left";
                cmd.target_velocity = 0;
                cmd.command_type = 6;  // 6 = park at 0 (slip-mode suppression)
                cmd.valid = true;
                bus_.update_command(cmd);
            } catch (const std::exception& e) {
                std::cerr << "[send_can_command_to_elmo] Left park-at-0 failed: " << e.what() << '\n';
            }
        }
        if (right_ready && !right_disabled_ && !right_initial_park_done_) {
            try {
                send_velocity_command(right_node_id_, 0);
                right_initial_park_done_ = true;
                ElmoCommand cmd;
                cmd.timestamp_ns = now_ns();
                cmd.foot = "Right";
                cmd.target_velocity = 0;
                cmd.command_type = 6;
                cmd.valid = true;
                bus_.update_command(cmd);
            } catch (const std::exception& e) {
                std::cerr << "[send_can_command_to_elmo] Right park-at-0 failed: " << e.what() << '\n';
            }
        }
    }

    const SystemSnapshot s = bus_.snapshot();

    if (left_ready && !left_disabled_) {
        const bool left_fault = s.status_left.valid && s.status_left.fault;
        if (left_fault && active_fault_foot_names_.insert("Left").second) {
            try {
                stop_and_reset_elmo(left_node_id_, "Left");
            } catch (const std::exception& e) {
                std::cerr << "[send_can_command_to_elmo] Left fault handling failed: " << e.what() << '\n';
            }
        } else if (!left_fault) {
            active_fault_foot_names_.erase("Left");
        }

        if (s.gait_left.valid) {
            process_foot(s.gait_left, left_node_id_, "Left", last_left_detection_count_);
        }
    }

    if (right_ready && !right_disabled_) {
        const bool right_fault = s.status_right.valid && s.status_right.fault;
        if (right_fault && active_fault_foot_names_.insert("Right").second) {
            try {
                stop_and_reset_elmo(right_node_id_, "Right");
            } catch (const std::exception& e) {
                std::cerr << "[send_can_command_to_elmo] Right fault handling failed: " << e.what() << '\n';
            }
        } else if (!right_fault) {
            active_fault_foot_names_.erase("Right");
        }

        if (s.gait_right.valid) {
            process_foot(s.gait_right, right_node_id_, "Right", last_right_detection_count_);
        }
    }
}

void SendCanCommandToElmoNode::process_foot(
    const GaitPhase& gait, int node_id, const std::string& foot, uint32_t& last_detection_count) {
    if (active_fault_foot_names_.count(foot) > 0) {
        return;
    }

    const bool external = (foot == "Left") ? left_external_active_ : right_external_active_;
    if (external) {
        last_detection_count = gait.detection_count;
        return;
    }

    if (suppress_gait_velocity_commands_.load(std::memory_order_acquire)) {
        last_detection_count = gait.detection_count;
        return;
    }

    if (gait.detection_count == last_detection_count) {
        return;
    }
    last_detection_count = gait.detection_count;

    int32_t target_velocity = 0;
    const auto it = velocity_map_.find(gait.phase);
    if (it != velocity_map_.end()) {
        target_velocity = it->second;
    }

    try {
        send_velocity_command(node_id, target_velocity);
    } catch (const std::exception& e) {
        // Transient CAN write failure (e.g. ENOBUFS). Don't add to
        // active_fault_foot_names_ — that set tracks ELMO-reported drive
        // faults and is only cleared by a status fault->no-fault transition,
        // which won't happen for a CAN socket hiccup. Just log and retry on
        // the next phase change.
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " command send failed: " << e.what() << '\n';
        return;
    }

    ElmoCommand cmd;
    cmd.timestamp_ns = now_ns();
    cmd.foot = foot;
    cmd.target_velocity = target_velocity;
    cmd.command_type = 0;
    cmd.valid = true;
    bus_.update_command(cmd);
}

void SendCanCommandToElmoNode::inject_velocity(const std::string& foot, int32_t velocity) {
    if (active_fault_foot_names_.count(foot) > 0) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " inject_velocity ignored: ELMO drive fault active\n";
        return;
    }
    const bool disabled = (foot == "Left") ? left_disabled_ : right_disabled_;
    if (disabled) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " inject_velocity ignored: drive disabled\n";
        return;
    }
    const bool ready = (foot == "Left")
                           ? left_ready_.load(std::memory_order_acquire)
                           : right_ready_.load(std::memory_order_acquire);
    if (!ready) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " inject_velocity ignored: drive not ready\n";
        return;
    }

    const int node_id = (foot == "Left") ? left_node_id_ : right_node_id_;
    try {
        send_velocity_command(node_id, velocity);
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " inject_velocity failed: " << e.what() << '\n';
        return;
    }

    if (foot == "Left") {
        left_external_active_ = true;
    } else {
        right_external_active_ = true;
    }

    ElmoCommand cmd;
    cmd.timestamp_ns = now_ns();
    cmd.foot = foot;
    cmd.target_velocity = velocity;
    cmd.command_type = 2;  // 2 = external injection (slip perturbation)
    cmd.valid = true;
    bus_.update_command(cmd);

    std::cout << "[send_can_command_to_elmo] " << foot
              << " inject_velocity sent: " << velocity
              << " (node " << node_id << ")\n";
    std::cout.flush();
}

void SendCanCommandToElmoNode::release_external_control(const std::string& foot) {
    if (foot == "Left") {
        left_external_active_ = false;
    } else {
        right_external_active_ = false;
    }

    if (active_fault_foot_names_.count(foot) > 0) {
        return;
    }
    const bool disabled = (foot == "Left") ? left_disabled_ : right_disabled_;
    if (disabled) {
        return;
    }
    const bool ready = (foot == "Left")
                           ? left_ready_.load(std::memory_order_acquire)
                           : right_ready_.load(std::memory_order_acquire);
    if (!ready) {
        return;
    }

    const SystemSnapshot s = bus_.snapshot();
    const GaitPhase& gait = (foot == "Left") ? s.gait_left : s.gait_right;
    if (!gait.valid) {
        return;
    }

    int32_t target_velocity = 0;
    const auto it = velocity_map_.find(gait.phase);
    if (it != velocity_map_.end()) {
        target_velocity = it->second;
    }

    const int node_id = (foot == "Left") ? left_node_id_ : right_node_id_;
    try {
        send_velocity_command(node_id, target_velocity);
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " release failed: " << e.what() << '\n';
        return;
    }

    ElmoCommand cmd;
    cmd.timestamp_ns = now_ns();
    cmd.foot = foot;
    cmd.target_velocity = target_velocity;
    cmd.command_type = 3;  // 3 = post-slip release back to gait map
    cmd.valid = true;
    bus_.update_command(cmd);
}

bool SendCanCommandToElmoNode::is_externally_controlled(const std::string& foot) const {
    return (foot == "Left") ? left_external_active_ : right_external_active_;
}

void SendCanCommandToElmoNode::disable_drive(int node_id, const std::string& foot) {
    try {
        send_velocity_command(node_id, 0);
        auto shutdown = create_sdo_download(
            node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SHUTDOWN_STATE, 2);
        can_socket_->send_message(shutdown.can_id, shutdown.data, shutdown.dlc);
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " disable failed: " << e.what() << '\n';
        return;
    }

    if (foot == "Left") {
        left_disabled_ = true;
        left_external_active_ = false;
    } else {
        right_disabled_ = true;
        right_external_active_ = false;
    }

    ElmoCommand cmd;
    cmd.timestamp_ns = now_ns();
    cmd.foot = foot;
    cmd.target_velocity = 0;
    cmd.command_type = 4;  // 4 = drive disabled (Shutdown)
    cmd.valid = true;
    bus_.update_command(cmd);

    std::cout << "[send_can_command_to_elmo] " << foot << " disabled\n";
    std::cout.flush();
}

void SendCanCommandToElmoNode::reenable_drive(int node_id, const std::string& foot) {
    try {
        initialize_elmo_driver(node_id, /*configure_pdos=*/false);
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " re-enable failed: " << e.what() << '\n';
        return;
    }

    if (foot == "Left") {
        left_disabled_ = false;
        left_initial_park_done_ = false;
    } else {
        right_disabled_ = false;
        right_initial_park_done_ = false;
    }

    ElmoCommand cmd;
    cmd.timestamp_ns = now_ns();
    cmd.foot = foot;
    cmd.target_velocity = 0;
    cmd.command_type = 5;  // 5 = drive re-enabled
    cmd.valid = true;
    bus_.update_command(cmd);

    std::cout << "[send_can_command_to_elmo] " << foot << " re-enabled\n";
    std::cout.flush();
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

void SendCanCommandToElmoNode::initialize_elmo_driver(int node_id, bool configure_pdos) {
    using namespace std::chrono_literals;

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

    auto fault_reset = create_sdo_download(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_FAULT_RESET, 2);
    can_socket_->send_message(fault_reset.can_id, fault_reset.data, fault_reset.dlc);
    std::this_thread::sleep_for(50ms);

    auto mode = create_sdo_download(node_id, CANOPEN_MODE_OF_OPERATION, 0, 3, 1);
    can_socket_->send_message(mode.can_id, mode.data, mode.dlc);
    std::this_thread::sleep_for(50ms);

    // Profile acceleration / deceleration. The base values come from config
    // (profile_acceleration_ / profile_deceleration_) and are written
    // independently to 0x6083 and 0x6084, so accel and decel can differ.
    // Slip mode wants a fast symmetric ramp so the motor reaches the commanded
    // slip velocity before the burst ends; when slip_profile_acceleration_ > 0
    // it overrides BOTH accel and decel. The override is consulted here (with
    // the same 50 ms inter-SDO spacing the rest of init uses) rather than from
    // the real-time tick path, where back-to-back SDO writes have been seen to
    // make the drive silently abort and ignore later target-velocity writes.
    //
    // Ordering note: set_slip_profile_acceleration() runs after the constructor
    // returns, but this init thread reads slip_profile_acceleration_ ~0.5 s into
    // init, so the slip app's post-construction store is observed in time. This
    // is order-dependent; if init timing ever changes, plumb the slip accel
    // through the constructor instead.
    const int32_t slip_accel = slip_profile_acceleration_.load(std::memory_order_acquire);
    const bool slip_override = slip_accel > 0;
    const uint32_t accel_value =
        slip_override ? static_cast<uint32_t>(slip_accel) : static_cast<uint32_t>(profile_acceleration_);
    const uint32_t decel_value =
        slip_override ? static_cast<uint32_t>(slip_accel) : static_cast<uint32_t>(profile_deceleration_);

    auto accel = create_sdo_download(node_id, 0x6083, 0, accel_value, 4);
    can_socket_->send_message(accel.can_id, accel.data, accel.dlc);
    std::this_thread::sleep_for(50ms);

    auto decel = create_sdo_download(node_id, 0x6084, 0, decel_value, 4);
    can_socket_->send_message(decel.can_id, decel.data, decel.dlc);
    std::this_thread::sleep_for(50ms);

    auto shutdown = create_sdo_download(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SHUTDOWN_STATE, 2);
    can_socket_->send_message(shutdown.can_id, shutdown.data, shutdown.dlc);
    std::this_thread::sleep_for(50ms);

    auto switch_on = create_sdo_download(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SWITCH_ON_STATE, 2);
    can_socket_->send_message(switch_on.can_id, switch_on.data, switch_on.dlc);
    std::this_thread::sleep_for(50ms);

    auto enable = create_sdo_download(node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_ENABLE_OPERATION_STATE, 2);
    can_socket_->send_message(enable.can_id, enable.data, enable.dlc);
    std::this_thread::sleep_for(50ms);
}

void SendCanCommandToElmoNode::configure_pdo_mapping(int node_id) {
    using namespace std::chrono_literals;

    // Configure two SYNC-triggered TPDOs per drive so motor feedback streams at
    // the SYNC rate (1 kHz) without per-value SDO request frames:
    //   TPDO1 (0x180+id): position 0x6064 (32b) + velocity 0x606C (32b) = 8 B
    //   TPDO2 (0x280+id): current 0x6078 (16b) + velocity demand 0x606B (32b)
    //                     + current demand 0x6074 (16b)                 = 8 B
    // TPDO2 packs the drive-internal command (demand) values alongside current
    // in the same 8-byte frame, so streaming them costs no extra bus bandwidth.
    // Statusword stays on the existing 2 Hz SDO poll for fault detection.
    //
    // Standard remap procedure per PDO: disable the PDO (COB-ID bit 31), set the
    // transmission type, clear the mapping count, write the mapping entries, set
    // the count, then re-enable the COB-ID. Writes are blind (fire-and-forget
    // with a short gap), matching the rest of init — the SDO responses are not
    // verified here. The inter-write gap keeps the drive's SDO server from
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
}

void SendCanCommandToElmoNode::send_velocity_command(int node_id, int32_t velocity) {
    auto vel = create_sdo_download(node_id, CANOPEN_TARGET_VELOCITY, 0, static_cast<uint32_t>(velocity), 4);
    can_socket_->send_message(vel.can_id, vel.data, vel.dlc);
}

void SendCanCommandToElmoNode::stop_and_reset_elmo(int node_id, const std::string& foot) {
    send_velocity_command(node_id, 0);

    ElmoCommand stop_cmd;
    stop_cmd.timestamp_ns = now_ns();
    stop_cmd.foot = foot;
    stop_cmd.target_velocity = 0;
    stop_cmd.command_type = 1;
    stop_cmd.valid = true;
    bus_.update_command(stop_cmd);

    // Bare Fault Reset leaves the drive in 'Switch On Disabled', so re-run the
    // full init (Fault Reset -> Shutdown -> Switch On -> Enable Operation) to
    // bring it back to 'Operation Enabled'. This blocks the loop for ~0.5 s
    // during recovery; that's acceptable for a fault-recovery path.
    std::cerr << "[send_can_command_to_elmo] re-arming " << foot
              << " drive (node " << node_id << ")\n";
    try {
        initialize_elmo_driver(node_id, /*configure_pdos=*/false);
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " drive re-armed\n";
    } catch (const std::exception& e) {
        std::cerr << "[send_can_command_to_elmo] " << foot
                  << " re-arm failed: " << e.what() << '\n';
    }
}

}  // namespace motorized_shoe
