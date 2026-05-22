#include "motorized_shoe/read_can_malfunction_from_elmo_node.hpp"

#include <iostream>

namespace motorized_shoe {

ReadCanMalfunctionFromElmoNode::ReadCanMalfunctionFromElmoNode(const Config& cfg, DataBus& bus)
    : bus_(bus),
      can_socket_(std::make_unique<CANSocket>(cfg.can_elmo_interface)),
      last_status_request_(std::chrono::steady_clock::now()) {
    elmo_nodes_.push_back({cfg.elmo_node_left, "Left"});
    elmo_nodes_.push_back({cfg.elmo_node_right, "Right"});
}

void ReadCanMalfunctionFromElmoNode::tick() {
    uint32_t can_id = 0;
    uint8_t data[8] = {0};
    size_t len = 0;

    try {
        while (can_socket_->recv_message(can_id, data, len, 0)) {
            if (can_id >= 0x580 && can_id <= 0x5FF) {
                process_sdo_response(can_id - 0x580, data, len);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[read_can_malfunction_from_elmo] receive failed: " << e.what() << '\n';
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_request_).count() >= 500) {
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
}

void ReadCanMalfunctionFromElmoNode::request_status_word(int node_id) {
    uint8_t msg_data[8] = {0};
    msg_data[0] = 0x40;
    msg_data[1] = 0x41;
    msg_data[2] = 0x60;
    msg_data[3] = 0x00;
    can_socket_->send_message(0x600 + static_cast<uint32_t>(node_id), msg_data, 4);
}

void ReadCanMalfunctionFromElmoNode::process_sdo_response(uint32_t node_id, const uint8_t* data, size_t len) {
    if (len < 4) {
        return;
    }

    const uint8_t cmd = data[0];
    const uint16_t index = static_cast<uint16_t>(data[1] | (data[2] << 8));
    const uint8_t subindex = data[3];

    if (index == CANOPEN_STATUS_WORD && subindex == 0 && len >= 8) {
        if (cmd == 0x60 || cmd == 0x43) {
            const uint16_t status_word = static_cast<uint16_t>(data[4] | (data[5] << 8));
            publish_status(node_id, status_word);
        }
    }

    if (cmd == 0x80 && len >= 8) {
        const uint32_t abort_code =
            static_cast<uint32_t>(data[4]) |
            (static_cast<uint32_t>(data[5]) << 8) |
            (static_cast<uint32_t>(data[6]) << 16) |
            (static_cast<uint32_t>(data[7]) << 24);
        publish_error(node_id, abort_code);
    }
}

void ReadCanMalfunctionFromElmoNode::publish_status(uint32_t node_id, uint16_t status_word) {
    std::string foot = "Unknown";
    for (const auto& node : elmo_nodes_) {
        if (static_cast<uint32_t>(node.node_id) == node_id) {
            foot = node.foot;
            break;
        }
    }

    ElmoStatus status;
    status.timestamp_ns = now_ns();
    status.foot = foot;
    status.status_word = status_word;
    status.ready_to_switch_on = (status_word & STATUS_READY_TO_SWITCH_ON) != 0;
    status.switched_on = (status_word & STATUS_SWITCHED_ON) != 0;
    status.operation_enabled = (status_word & STATUS_OPERATION_ENABLED) != 0;
    status.fault = (status_word & STATUS_FAULT) != 0;
    status.motor_enabled = status.switched_on && status.operation_enabled;
    status.error_message = status.fault ? "ELMO Fault detected" : "";
    status.valid = true;

    bus_.update_status(status);
}

void ReadCanMalfunctionFromElmoNode::publish_error(uint32_t node_id, uint32_t abort_code) {
    std::string foot = "Unknown";
    for (const auto& node : elmo_nodes_) {
        if (static_cast<uint32_t>(node.node_id) == node_id) {
            foot = node.foot;
            break;
        }
    }

    ElmoStatus status;
    status.timestamp_ns = now_ns();
    status.foot = foot;
    status.fault = true;
    status.error_message = "SDO abort code: 0x" + to_hex_string(abort_code);
    status.valid = true;

    bus_.update_status(status);
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
