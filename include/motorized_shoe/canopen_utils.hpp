#ifndef MOTORIZED_SHOE_CANOPEN_UTILS_HPP
#define MOTORIZED_SHOE_CANOPEN_UTILS_HPP

#include <cstddef>
#include <cstdint>

namespace motorized_shoe {

#define CANOPEN_NMT_START 0x01
#define CANOPEN_NMT_PREOP 0x80

// SYNC object COB-ID. The master transmits this (0 data bytes) once per control
// loop; SYNC-triggered TPDOs (transmission type 1) fire on receipt.
#define CANOPEN_SYNC_COB_ID 0x80

// TPDO communication + mapping parameter objects (TPDO1, TPDO2).
//   comm sub1 = COB-ID (bit 31 set = PDO disabled while editing)
//   comm sub2 = transmission type (1 = on every SYNC)
//   map  sub0 = number of mapped entries; sub1..n = index<<16|subindex<<8|bits
#define CANOPEN_TPDO1_COMM 0x1800
#define CANOPEN_TPDO2_COMM 0x1801
#define CANOPEN_TPDO1_MAP 0x1A00
#define CANOPEN_TPDO2_MAP 0x1A01
#define CANOPEN_TPDO1_COB_BASE 0x180
#define CANOPEN_TPDO2_COB_BASE 0x280

#define CANOPEN_MODE_OF_OPERATION 0x6060
#define CANOPEN_TARGET_VELOCITY 0x60FF
#define CANOPEN_STATUS_WORD 0x6041
#define CANOPEN_CONTROL_WORD 0x6040

// CiA-402 motor feedback objects (read back via SDO upload).
#define CANOPEN_POSITION_ACTUAL 0x6064  // INT32, counts
#define CANOPEN_VELOCITY_ACTUAL 0x606C  // INT32, counts/sec
#define CANOPEN_CURRENT_ACTUAL 0x6078   // INT16, per-mille of rated current

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

// Expedited SDO upload (read) request: command specifier 0x40, no data. The
// drive replies on 0x580+node_id with the object's value and a 0x4x command
// byte encoding the data width.
CANopenMessage create_sdo_upload(uint32_t node_id, uint16_t index, uint8_t subindex);

CANopenMessage create_nmt_message(uint32_t node_id, uint8_t command);

// SYNC frame: COB-ID 0x80, zero data bytes.
CANopenMessage create_sync_message();

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_CANOPEN_UTILS_HPP
