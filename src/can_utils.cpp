#include "motorized_shoe/can_utils.hpp"

#include <cerrno>
#include <cstring>
#include <poll.h>
#include <sstream>
#include <unistd.h>

namespace motorized_shoe {

CANSocket::CANSocket(const std::string& interface_name) : sock_(-1) {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        throw std::runtime_error("Failed to create CAN socket");
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        close(sock_);
        throw std::runtime_error("Failed to get CAN interface index: " + interface_name);
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr_), sizeof(addr_)) < 0) {
        close(sock_);
        throw std::runtime_error("Failed to bind CAN socket to interface: " + interface_name);
    }
}

CANSocket::~CANSocket() {
    if (sock_ >= 0) {
        close(sock_);
    }
}

void CANSocket::send_message(uint32_t can_id, const uint8_t* data, size_t len) {
    if (len > 8) {
        throw std::runtime_error("CAN frame data length exceeded (max 8 bytes)");
    }

    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = static_cast<__u8>(len);
    std::memset(frame.data, 0, sizeof(frame.data));
    std::memcpy(frame.data, data, len);

    if (write(sock_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
        std::ostringstream oss;
        oss << "Failed to send CAN message (can_id=0x" << std::hex << can_id
            << ", errno=" << std::dec << errno << ": " << std::strerror(errno) << ")";
        throw std::runtime_error(oss.str());
    }
}

bool CANSocket::recv_message(uint32_t& can_id, uint8_t* data, size_t& len, int timeout_ms) {
    struct pollfd fds[1];
    fds[0].fd = sock_;
    fds[0].events = POLLIN;

    const int poll_result = poll(fds, 1, timeout_ms);
    if (poll_result < 0) {
        std::ostringstream oss;
        oss << "Poll error on CAN socket (errno=" << errno
            << ": " << std::strerror(errno) << ")";
        throw std::runtime_error(oss.str());
    }
    if (poll_result == 0) {
        return false;
    }

    struct can_frame frame;
    const int nbytes = read(sock_, &frame, sizeof(frame));
    if (nbytes < 0) {
        std::ostringstream oss;
        oss << "Failed to read CAN message (errno=" << errno
            << ": " << std::strerror(errno) << ")";
        throw std::runtime_error(oss.str());
    }

    can_id = frame.can_id;
    len = frame.can_dlc;
    std::memcpy(data, frame.data, len);
    return true;
}

}  // namespace motorized_shoe
