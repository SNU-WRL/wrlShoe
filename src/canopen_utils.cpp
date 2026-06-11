#include "motorized_shoe/canopen_utils.hpp"

namespace motorized_shoe {

CANopenMessage create_sdo_download(uint32_t node_id, uint16_t index,
                                   uint8_t subindex, uint32_t value, int length) {
    CANopenMessage msg;
    msg.can_id = encode_canopen_sdo_id(node_id);

    uint8_t cs = 0x23;
    if (length == 1) {
        cs = 0x2F;
    } else if (length == 2) {
        cs = 0x2B;
    }

    msg.data[0] = cs;
    msg.data[1] = static_cast<uint8_t>(index & 0xFF);
    msg.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    msg.data[3] = subindex;

    for (int i = 0; i < length && i < 4; ++i) {
        msg.data[4 + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
    }
    for (int i = length; i < 4; ++i) {
        msg.data[4 + i] = 0;
    }

    msg.dlc = 8;
    return msg;
}

CANopenMessage create_sdo_upload(uint32_t node_id, uint16_t index, uint8_t subindex) {
    CANopenMessage msg;
    msg.can_id = encode_canopen_sdo_id(node_id);

    msg.data[0] = 0x40;  // initiate upload (read) request
    msg.data[1] = static_cast<uint8_t>(index & 0xFF);
    msg.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    msg.data[3] = subindex;
    // data[4..7] left zero (unused in the request)

    msg.dlc = 8;
    return msg;
}

CANopenMessage create_nmt_message(uint32_t node_id, uint8_t command) {
    CANopenMessage msg;
    msg.can_id = 0x000;
    msg.data[0] = command;
    msg.data[1] = static_cast<uint8_t>(node_id);
    msg.dlc = 2;
    return msg;
}

CANopenMessage create_sync_message() {
    CANopenMessage msg;
    msg.can_id = CANOPEN_SYNC_COB_ID;
    msg.dlc = 0;
    return msg;
}

}  // namespace motorized_shoe
