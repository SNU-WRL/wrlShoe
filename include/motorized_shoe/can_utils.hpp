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

namespace motorized_shoe {

class CANSocket {
public:
    explicit CANSocket(const std::string& interface_name);
    ~CANSocket();

    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;

    void send_message(uint32_t can_id, const uint8_t* data, size_t len);
    bool recv_message(uint32_t& can_id, uint8_t* data, size_t& len, int timeout_ms = 0);

private:
    int sock_;
    struct sockaddr_can addr_;
};

inline int16_t get_int16(const uint8_t* buf) {
    return static_cast<int16_t>((buf[1] << 8) | buf[0]);
}

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_CAN_UTILS_HPP
