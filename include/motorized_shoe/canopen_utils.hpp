#ifndef MOTORIZED_SHOE_CANOPEN_UTILS_HPP
#define MOTORIZED_SHOE_CANOPEN_UTILS_HPP

#include <cstddef>
#include <cstdint>
#include <string>

#include "motorized_shoe/can_utils.hpp"

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

// CiA-402 drive-internal command (demand) objects: the output of the drive's
// own control loops -- what the ELMO is commanding the motor, after profile
// shaping (vs. our 0x60FF target velocity and the actual feedback above).
#define CANOPEN_VELOCITY_DEMAND 0x606B  // INT32, counts/sec (velocity setpoint)
// 0x6074 is CiA-402 "torque demand", but on a current-mode ELMO drive it IS the
// commanded current (torque is produced by q-axis current). It is normalized to
// per-mille of rated torque, the same scale as 0x6078 current actual (per-mille
// of rated current), so the two are directly comparable as command vs. measured.
#define CANOPEN_CURRENT_DEMAND 0x6074   // INT16, per-mille of rated torque/current

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

// Elmo native-interpreter access via the CiA-301 OS-command object (0x1023).
//
// The drive's trajectory generator ramps with the native SD ("stop
// deceleration") parameter, NOT with the DS402 profile accel/decel objects and
// NOT with native AC/DC. Verified on hardware (2026-07-16, node 127, UM=5,
// CANopen Profile Velocity): with 0x6083/0x6084 = 1e7 confirmed written AND
// AC = 5e6..1e7 read back correctly, the velocity-demand slope stayed at
// exactly 1e6 (SD's flash default) in both directions; setting `SD=5000000`
// through 0x1023 immediately moved the measured ramp to ~5e6. SD governs both
// the accel and decel side of the PV profile. AC/DC/0x6083/0x6084 are stored
// and read back faithfully but never reach the trajectory generator in this
// unit mode, so a readback of those objects proves nothing about the ramp.
// SD set this way is volatile (a power cycle restores the flash value, 1e6 on
// our drives), so it must be re-written on every init.
//
// Protocol: write the ASCII command to 0x1023:01, poll the status byte at
// 0x1023:02 until it leaves 0xFF (executing), then read the ASCII reply from
// 0x1023:03. Commands and replies longer than 4 bytes use segmented SDO
// transfers. Frames on 0x580+node that belong to other exchanges (e.g. the
// statusword poll) are skipped; if such a request aborts our in-progress
// segmented transfer on the drive side, the exchange fails and the caller
// should retry.
#define CANOPEN_OS_COMMAND 0x1023

struct ElmoOsCommandResult {
    bool ok = false;        // exchange completed and the drive reported success
    uint8_t status = 0xFF;  // 0/1 = OK (no reply / reply), 2/3 = command error
    std::string reply;      // ASCII reply, trailing NULs/';' stripped
    std::string error;      // failure description when !ok
};

// Runs one full OS-command exchange (single attempt, no internal retry).
// recv_timeout_ms bounds each individual SDO response wait.
ElmoOsCommandResult elmo_os_command(CANSocket& sock, int node_id, const std::string& command,
                                    int recv_timeout_ms = 100);

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_CANOPEN_UTILS_HPP
