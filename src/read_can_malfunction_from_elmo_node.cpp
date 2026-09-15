#include "motorized_shoe/read_can_malfunction_from_elmo_node.hpp"

#include <iostream>

namespace motorized_shoe {

const char* canopen_error_code_label(uint16_t code) {
    // CiA-301 / CiA-402 generic error codes (the Elmo manual lists the exact
    // drive condition behind each; the MF register has the fine detail).
    switch (code) {
        case 0x0000: return "no error / error reset";
        case 0x1000: return "generic error";
        case 0x2310: return "continuous over-current";
        case 0x2311: return "short circuit / over-current";
        case 0x2320: return "short circuit";
        case 0x2340: return "short circuit / earth leakage";
        case 0x3110: return "mains over-voltage";
        case 0x3120: return "mains under-voltage";
        case 0x3210: return "DC link over-voltage";
        case 0x3220: return "DC link under-voltage";
        case 0x4210: return "over-temperature (device)";
        case 0x4310: return "over-temperature (drive)";
        case 0x5280: return "safe torque off / enable input";
        case 0x5441: return "hardware enable / limit switch";
        case 0x6180: return "software error / internal";
        case 0x6320: return "parameter error";
        case 0x7121: return "motor blocked / stuck";
        case 0x7300: return "sensor / feedback fault";
        case 0x7305: return "incremental encoder fault";
        case 0x7380: return "position sensor fault";
        case 0x8110: return "CAN overrun";
        case 0x8120: return "CAN error passive";
        case 0x8130: return "CAN heartbeat / life guard";
        case 0x8140: return "CAN recovered from bus-off";
        case 0x8210: return "PDO not processed (length)";
        case 0x8480: return "velocity: speed controller / over-speed";
        case 0x8611: return "following error (position)";
        case 0x8680: return "velocity tracking error";
        case 0xFF00: return "manufacturer-specific";
        default:
            if (code >= 0xFF00) return "manufacturer-specific";
            return "unknown";
    }
}

// CiA-301 0x81xx codes are communication-layer notices (CAN overrun, error
// passive, recovered from bus-off, PDO length). On the Elmo drives they do NOT
// drop the motor: on 2026-09-15 a 0x8110 arrived while the statusword still
// read 0x1637 (Operation Enabled). Treating them as faults made the control
// thread re-arm a healthy drive (~100 ms with the motor off). The one
// exception is 0x8130 (heartbeat / life-guard event), which the drive is
// configured to act on, so it stays a fault.
bool canopen_emcy_is_warning(uint16_t code) {
    return (code & 0xFF00) == 0x8100 && code != 0x8130;
}

ReadCanMalfunctionFromElmoNode::ReadCanMalfunctionFromElmoNode(const Config& cfg, DataBus& bus)
    : bus_(bus),
      can_socket_(std::make_unique<CANSocket>(cfg.can_elmo_interface)),
      last_status_request_(std::chrono::steady_clock::now()),
      status_poll_ms_((cfg.status_poll_ms > 0) ? cfg.status_poll_ms : 500) {
    elmo_nodes_.push_back({cfg.elmo_node_left, "Left", ElmoStatus{}, false});
    elmo_nodes_.push_back({cfg.elmo_node_right, "Right", ElmoStatus{}, false});
    for (auto& n : elmo_nodes_) {
        n.status.foot = n.foot;
    }

    motor_left_.foot = "Left";
    motor_right_.foot = "Right";

    // Only the frames this node parses: EMCY (0x080+id), TPDO1 (0x180+id),
    // TPDO2 (0x280+id) and SDO responses (0x580+id). Everything else (our
    // own SDO requests echoed back, NMT, heartbeat) is dropped in the kernel.
    can_socket_->set_filters({{0x080, 0x780}, {0x180, 0x780}, {0x280, 0x780}, {0x580, 0x780}});
}

void ReadCanMalfunctionFromElmoNode::tick() {
    uint32_t can_id = 0;
    uint8_t data[8] = {0};
    size_t len = 0;

    try {
        while (can_socket_->recv_message(can_id, data, len, 0)) {
            if (can_id >= 0x180 && can_id <= 0x1FF) {
                process_tpdo1(can_id - CANOPEN_TPDO1_COB_BASE, data, len);
            } else if (can_id >= 0x280 && can_id <= 0x2FF) {
                process_tpdo2(can_id - CANOPEN_TPDO2_COB_BASE, data, len);
            } else if (can_id >= 0x580 && can_id <= 0x5FF) {
                process_sdo_response(can_id - 0x580, data, len);
            } else if (can_id >= 0x081 && can_id <= 0x0FF) {
                process_emcy(can_id - 0x080, data, len);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[read_can_malfunction_from_elmo] receive failed: " << e.what() << '\n';
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_request_).count() >= status_poll_ms_) {
        for (const auto& node : elmo_nodes_) {
            try {
                request_status_word(node.node_id);
            } catch (const std::exception& e) {
                std::cerr << "[read_can_malfunction_from_elmo] status request failed for "
                          << node.foot << ": " << e.what() << '\n';
            }
        }
        last_status_request_ = now;
    }

    // Trigger the next round of feedback TPDOs. The drives transmit on receipt;
    // those frames are drained at the top of the next tick.
    send_sync();
}

void ReadCanMalfunctionFromElmoNode::send_sync() {
    try {
        auto sync = create_sync_message();
        can_socket_->send_message(sync.can_id, sync.data, sync.dlc);
    } catch (const std::exception& e) {
        std::cerr << "[read_can_malfunction_from_elmo] SYNC send failed: " << e.what() << '\n';
    }
}

void ReadCanMalfunctionFromElmoNode::request_status_word(int node_id) {
    auto req = create_sdo_upload(static_cast<uint32_t>(node_id), CANOPEN_STATUS_WORD, 0);
    can_socket_->send_message(req.can_id, req.data, req.dlc);
}

void ReadCanMalfunctionFromElmoNode::request_error_code(int node_id) {
    auto req = create_sdo_upload(static_cast<uint32_t>(node_id), CANOPEN_ERROR_CODE, 0);
    can_socket_->send_message(req.can_id, req.data, req.dlc);
}

ReadCanMalfunctionFromElmoNode::ElmoNode* ReadCanMalfunctionFromElmoNode::node_for(uint32_t node_id) {
    for (auto& node : elmo_nodes_) {
        if (static_cast<uint32_t>(node.node_id) == node_id) {
            return &node;
        }
    }
    return nullptr;
}

void ReadCanMalfunctionFromElmoNode::process_sdo_response(uint32_t node_id, const uint8_t* data, size_t len) {
    if (len < 4) {
        return;
    }
    ElmoNode* node = node_for(node_id);
    if (node == nullptr) {
        return;
    }

    const uint8_t cmd = data[0];
    const uint16_t index = static_cast<uint16_t>(data[1] | (data[2] << 8));
    const uint8_t subindex = data[3];

    if (cmd == 0x80 && len >= 8) {
        // SDO abort. This is a PROTOCOL error on one SDO transaction (ours or
        // the command node's -- every raw socket on can0 sees every frame),
        // not a drive fault, so it must not flip the fault flag: doing so used
        // to trigger a full blocking drive re-init from a mere aborted write.
        const uint32_t abort_code =
            static_cast<uint32_t>(data[4]) |
            (static_cast<uint32_t>(data[5]) << 8) |
            (static_cast<uint32_t>(data[6]) << 16) |
            (static_cast<uint32_t>(data[7]) << 24);
        std::cerr << "[read_can_malfunction_from_elmo] " << node->foot << " SDO abort on 0x"
                  << to_hex_string(index).substr(4) << ":" << static_cast<int>(subindex)
                  << " code 0x" << to_hex_string(abort_code) << '\n';
        return;
    }

    // Any upload (read) response has SCS 010b, i.e. (cmd & 0xE0) == 0x40
    // (0x43/0x47/0x4B/0x4F depending on the object width).
    if ((cmd & 0xE0) != 0x40) {
        return;
    }

    if (index == CANOPEN_STATUS_WORD && subindex == 0 && len >= 6) {
        const uint16_t status_word = static_cast<uint16_t>(data[4] | (data[5] << 8));
        publish_status(*node, status_word);
    } else if (index == CANOPEN_ERROR_CODE && subindex == 0 && len >= 6) {
        const uint16_t code = static_cast<uint16_t>(data[4] | (data[5] << 8));
        node->error_code_requested = false;
        if (code != node->status.error_code) {
            node->status.error_code = code;
            node->status.timestamp_ns = now_ns();
            bus_.update_status(node->status);
        }
        std::cerr << "[read_can_malfunction_from_elmo] " << node->foot
                  << " error code (0x603F) = 0x" << to_hex_string(code).substr(4)
                  << " (" << canopen_error_code_label(code) << ")\n";
    }
}

void ReadCanMalfunctionFromElmoNode::process_emcy(uint32_t node_id, const uint8_t* data, size_t len) {
    // CiA-301 EMCY: [0:2] error code, [2] error register, [3:8] manufacturer
    // specific. The drive sends one the instant it faults (and one with code
    // 0 when the error is reset), so this is the fastest fault notification
    // we have -- the statusword poll only runs every status_poll_ms.
    ElmoNode* node = node_for(node_id);
    if (node == nullptr || len < 3) {
        return;
    }
    const uint16_t code = static_cast<uint16_t>(data[0] | (data[1] << 8));
    const uint8_t error_register = data[2];

    std::string mfr;
    for (size_t i = 3; i < len && i < 8; ++i) {
        mfr += to_hex_string(data[i]).substr(6);
        if (i + 1 < len) mfr += ' ';
    }
    std::cerr << "[read_can_malfunction_from_elmo] " << node->foot << " EMCY error 0x"
              << to_hex_string(code).substr(4) << " (" << canopen_error_code_label(code)
              << ") error-register 0x" << to_hex_string(error_register).substr(6)
              << " mfr[" << mfr << "]\n";

    node->status.timestamp_ns = now_ns();
    node->status.valid = true;
    if (code != 0 && canopen_emcy_is_warning(code)) {
        // Warning only: keep the code for the log / CSV column but leave
        // `fault` to the statusword poll, which is authoritative for the
        // drive's actual CiA-402 state. No recovery is triggered.
        node->status.error_code = code;
        std::cerr << "[read_can_malfunction_from_elmo] " << node->foot
                  << " EMCY 0x" << to_hex_string(code).substr(4)
                  << " is a communication warning, not a drive fault; no recovery\n";
    } else if (code != 0) {
        node->status.error_code = code;
        node->status.fault = true;
        node->status.motor_enabled = false;
        node->status.operation_enabled = false;
        node->status.error_message = std::string("EMCY 0x") + to_hex_string(code).substr(4) +
                                     " " + canopen_error_code_label(code);
        // An EMCY beats the poll; the next statusword read confirms and, once
        // the fault is reset, clears the flag.
        node->error_code_requested = true;  // no need for the 0x603F read
    } else {
        node->status.error_message = "";
        // Do not clear `fault` here: the statusword poll is authoritative for
        // the drive's actual CiA-402 state.
    }
    bus_.update_status(node->status);
}

void ReadCanMalfunctionFromElmoNode::publish_status(ElmoNode& node, uint16_t status_word) {
    ElmoStatus& status = node.status;
    const bool was_fault = status.valid && status.fault;

    status.timestamp_ns = now_ns();
    status.status_word = status_word;
    status.ready_to_switch_on = (status_word & STATUS_READY_TO_SWITCH_ON) != 0;
    status.switched_on = (status_word & STATUS_SWITCHED_ON) != 0;
    status.operation_enabled = (status_word & STATUS_OPERATION_ENABLED) != 0;
    status.fault = (status_word & STATUS_FAULT) != 0;
    status.motor_enabled = status.switched_on && status.operation_enabled;
    if (status.fault) {
        if (status.error_message.empty()) {
            status.error_message = "ELMO Fault detected";
        }
    } else {
        status.error_message = "";
    }
    status.valid = true;

    if (status.fault && !was_fault) {
        std::cerr << "[read_can_malfunction_from_elmo] " << node.foot
                  << " FAULT (statusword 0x" << to_hex_string(status_word).substr(4) << ")\n";
        if (!node.error_code_requested) {
            // No EMCY told us why: ask the drive for its error code.
            try {
                request_error_code(node.node_id);
                node.error_code_requested = true;
            } catch (const std::exception& e) {
                std::cerr << "[read_can_malfunction_from_elmo] error-code request failed for "
                          << node.foot << ": " << e.what() << '\n';
            }
        }
    } else if (!status.fault && was_fault) {
        std::cerr << "[read_can_malfunction_from_elmo] " << node.foot
                  << " fault cleared (statusword 0x" << to_hex_string(status_word).substr(4)
                  << (status.operation_enabled ? ", Operation Enabled" : ", NOT enabled") << ")\n";
        node.error_code_requested = false;
        // Keep error_code as "last fault reason" for the log; a new fault
        // overwrites it.
    }

    bus_.update_status(status);
}

ElmoMotorInfo* ReadCanMalfunctionFromElmoNode::motor_info_for(uint32_t node_id) {
    for (const auto& node : elmo_nodes_) {
        if (static_cast<uint32_t>(node.node_id) == node_id) {
            return (node.foot == "Left") ? &motor_left_ : &motor_right_;
        }
    }
    return nullptr;
}

void ReadCanMalfunctionFromElmoNode::process_tpdo1(
    uint32_t node_id, const uint8_t* data, size_t len) {
    // TPDO1 payload: position (INT32) + velocity (INT32), little-endian.
    if (len < 8) {
        return;
    }
    ElmoMotorInfo* motor = motor_info_for(node_id);
    if (motor == nullptr) {
        return;
    }

    motor->position = static_cast<int32_t>(
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24));
    motor->velocity = static_cast<int32_t>(
        static_cast<uint32_t>(data[4]) |
        (static_cast<uint32_t>(data[5]) << 8) |
        (static_cast<uint32_t>(data[6]) << 16) |
        (static_cast<uint32_t>(data[7]) << 24));
    motor->position_valid = true;
    motor->velocity_valid = true;
    motor->timestamp_ns = now_ns();
    motor->valid = true;
    bus_.update_motor_info(*motor);
}

void ReadCanMalfunctionFromElmoNode::process_tpdo2(
    uint32_t node_id, const uint8_t* data, size_t len) {
    // TPDO2 payload: current (INT16) + velocity demand (INT32) + current demand
    // (INT16), little-endian. Bytes: [0:2] current, [2:6] vel demand, [6:8]
    // current demand. Current is parsed if at least 2 bytes arrive; the demand
    // fields require the full 8-byte frame.
    if (len < 2) {
        return;
    }
    ElmoMotorInfo* motor = motor_info_for(node_id);
    if (motor == nullptr) {
        return;
    }

    motor->current = static_cast<int16_t>(data[0] | (data[1] << 8));
    motor->current_valid = true;
    if (len >= 8) {
        motor->velocity_demand = static_cast<int32_t>(
            static_cast<uint32_t>(data[2]) |
            (static_cast<uint32_t>(data[3]) << 8) |
            (static_cast<uint32_t>(data[4]) << 16) |
            (static_cast<uint32_t>(data[5]) << 24));
        motor->current_demand = static_cast<int16_t>(data[6] | (data[7] << 8));
        motor->velocity_demand_valid = true;
        motor->current_demand_valid = true;
    }
    motor->timestamp_ns = now_ns();
    motor->valid = true;
    bus_.update_motor_info(*motor);
}

std::string ReadCanMalfunctionFromElmoNode::to_hex_string(uint32_t value) const {
    const char* hex = "0123456789ABCDEF";
    std::string result;
    for (int i = 7; i >= 0; --i) {
        result += hex[(value >> (4 * i)) & 0xF];
    }
    return result;
}

}  // namespace motorized_shoe
