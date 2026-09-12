#ifndef MOTORIZED_SHOE_CAN_UTILS_HPP
#define MOTORIZED_SHOE_CAN_UTILS_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace motorized_shoe {

// One kernel-side receive filter: a frame is accepted when
// (frame.can_id & mask) == (id & mask).
struct CanIdFilter {
    uint32_t id;
    uint32_t mask;
};

class CANSocket {
public:
    explicit CANSocket(const std::string& interface_name);
    ~CANSocket();

    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;

    void send_message(uint32_t can_id, const uint8_t* data, size_t len);
    bool recv_message(uint32_t& can_id, uint8_t* data, size_t& len, int timeout_ms = 0);

    // Install kernel receive filters (CAN_RAW_FILTER). Every raw socket bound
    // to an interface otherwise receives EVERY frame on that bus, including
    // the 4 kHz TPDO stream and the other nodes' SDO traffic, and a socket
    // that is not drained every tick fills its receive buffer with frames it
    // will never look at. An empty list restores accept-all.
    void set_filters(const std::vector<CanIdFilter>& filters);

    // Discard everything currently queued on the socket. Used before an SDO
    // exchange so a stale response from an earlier transaction cannot be
    // mistaken for the reply to this one.
    void drain();

private:
    int sock_;
    struct sockaddr_can addr_;
};

inline int16_t get_int16(const uint8_t* buf) {
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_CAN_UTILS_HPP
