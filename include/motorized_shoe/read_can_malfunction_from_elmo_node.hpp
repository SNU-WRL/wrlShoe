#ifndef MOTORIZED_SHOE_READ_CAN_MALFUNCTION_FROM_ELMO_NODE_HPP
#define MOTORIZED_SHOE_READ_CAN_MALFUNCTION_FROM_ELMO_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "motorized_shoe/can_utils.hpp"
#include "motorized_shoe/canopen_utils.hpp"
#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"

namespace motorized_shoe {

class ReadCanMalfunctionFromElmoNode {
public:
    ReadCanMalfunctionFromElmoNode(const Config& cfg, DataBus& bus);
    void tick();

private:
    struct ElmoNode {
        int node_id;
        std::string foot;
    };

    void request_status_word(int node_id);
    void send_sync();
    void process_sdo_response(uint32_t node_id, const uint8_t* data, size_t len);
    void process_tpdo1(uint32_t node_id, const uint8_t* data, size_t len);
    void process_tpdo2(uint32_t node_id, const uint8_t* data, size_t len);
    void publish_status(uint32_t node_id, uint16_t status_word);
    void publish_error(uint32_t node_id, uint32_t abort_code);
    std::string to_hex_string(uint32_t value) const;
    ElmoMotorInfo* motor_info_for(uint32_t node_id);

    DataBus& bus_;
    std::unique_ptr<CANSocket> can_socket_;
    std::vector<ElmoNode> elmo_nodes_;
    std::chrono::steady_clock::time_point last_status_request_;

    // Motor feedback arrives via SYNC-triggered TPDOs (configured at init):
    //   TPDO1 (0x180+id): position + velocity   TPDO2 (0x280+id): current
    // We emit one SYNC per tick (1 kHz); the drives' TPDOs from the previous
    // SYNC are drained at the top of the next tick. Per-node working copies
    // accumulate the fields and are republished on each update.
    ElmoMotorInfo motor_left_;
    ElmoMotorInfo motor_right_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_READ_CAN_MALFUNCTION_FROM_ELMO_NODE_HPP
