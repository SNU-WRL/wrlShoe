#ifndef MOTORIZED_SHOE_CANOPEN_UTILS_HPP
#define MOTORIZED_SHOE_CANOPEN_UTILS_HPP

#include <cstddef>
#include <cstdint>

namespace motorized_shoe {

#define CANOPEN_NMT_START 0x01

#define CANOPEN_MODE_OF_OPERATION 0x6060
#define CANOPEN_TARGET_VELOCITY 0x60FF
#define CANOPEN_STATUS_WORD 0x6041
#define CANOPEN_CONTROL_WORD 0x6040

#define CANOPEN_SHUTDOWN_STATE 0x0006
#define CANOPEN_SWITCH_ON_STATE 0x0007
#define CANOPEN_ENABLE_OPERATION_STATE 0x000F
#define CANOPEN_FAULT_RESET 0x0080

#define STATUS_READY_TO_SWITCH_ON (1 << 0)
#define STATUS_SWITCHED_ON (1 << 1)
#define STATUS_OPERATION_ENABLED (1 << 2)
#define STATUS_FAULT (1 << 3)

struct CANopenMessage {
    uint32_t can_id = 0;
    uint8_t data[8] = {0};
    size_t dlc = 0;
};

inline uint32_t encode_canopen_sdo_id(uint32_t node_id) {
    return 0x600 + node_id;
}

inline uint32_t encode_canopen_sdo_rx_id(uint32_t node_id) {
    return 0x580 + node_id;
}

CANopenMessage create_sdo_download(uint32_t node_id, uint16_t index,
                                   uint8_t subindex, uint32_t value, int length);

CANopenMessage create_nmt_message(uint32_t node_id, uint8_t command);

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_CANOPEN_UTILS_HPP
