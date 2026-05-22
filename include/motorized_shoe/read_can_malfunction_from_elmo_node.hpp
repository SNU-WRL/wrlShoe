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
    void process_sdo_response(uint32_t node_id, const uint8_t* data, size_t len);
    void publish_status(uint32_t node_id, uint16_t status_word);
    void publish_error(uint32_t node_id, uint32_t abort_code);
    std::string to_hex_string(uint32_t value) const;

    DataBus& bus_;
    std::unique_ptr<CANSocket> can_socket_;
    std::vector<ElmoNode> elmo_nodes_;
    std::chrono::steady_clock::time_point last_status_request_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_READ_CAN_MALFUNCTION_FROM_ELMO_NODE_HPP
