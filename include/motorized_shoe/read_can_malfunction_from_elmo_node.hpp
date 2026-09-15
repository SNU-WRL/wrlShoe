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

// Human-readable label for a CiA-301/402 error code (EMCY / 0x603F). Returns
// "manufacturer-specific" / "unknown" for codes outside the standard table.
const char* canopen_error_code_label(uint16_t code);
// True for CiA-301 0x81xx communication notices (except 0x8130 heartbeat) that
// the drive reports without dropping the motor; they must not trigger recovery.
bool canopen_emcy_is_warning(uint16_t code);

class ReadCanMalfunctionFromElmoNode {
public:
    ReadCanMalfunctionFromElmoNode(const Config& cfg, DataBus& bus);
    void tick();

private:
    struct ElmoNode {
        int node_id;
        std::string foot;
        // Working copy of the published status so fields set by different
        // sources (statusword poll, EMCY, 0x603F read) accumulate.
        ElmoStatus status;
        bool error_code_requested = false;
    };

    void request_status_word(int node_id);
    void request_error_code(int node_id);
    void send_sync();
    void process_sdo_response(uint32_t node_id, const uint8_t* data, size_t len);
    void process_emcy(uint32_t node_id, const uint8_t* data, size_t len);
    void process_tpdo1(uint32_t node_id, const uint8_t* data, size_t len);
    void process_tpdo2(uint32_t node_id, const uint8_t* data, size_t len);
    void publish_status(ElmoNode& node, uint16_t status_word);
    std::string to_hex_string(uint32_t value) const;
    ElmoNode* node_for(uint32_t node_id);
    ElmoMotorInfo* motor_info_for(uint32_t node_id);

    DataBus& bus_;
    std::unique_ptr<CANSocket> can_socket_;
    std::vector<ElmoNode> elmo_nodes_;
    std::chrono::steady_clock::time_point last_status_request_;
    int status_poll_ms_;

    // Motor feedback arrives via SYNC-triggered TPDOs (configured at init):
    //   TPDO1 (0x180+id): position + velocity   TPDO2 (0x280+id): current
    // We emit one SYNC every sync_every_ticks_ ticks (elmo_sync_period_ms
    // at loop_frequency_hz); the drives' TPDOs from the previous SYNC are
    // drained at the top of the next tick. Per-node working copies
    // accumulate the fields and are republished on each update.
    int sync_every_ticks_;
    uint64_t tick_count_ = 0;
    ElmoMotorInfo motor_left_;
    ElmoMotorInfo motor_right_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_READ_CAN_MALFUNCTION_FROM_ELMO_NODE_HPP
