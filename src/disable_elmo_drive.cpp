// Sends a shutdown control word to disable the ELMO drive(s).
//
// Usage:
//   sudo ./build/disable_elmo_drive [--iface can0]
//        [--left 127] [--right 126] [--side both|left|right]

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "motorized_shoe/can_utils.hpp"
#include "motorized_shoe/canopen_utils.hpp"

using namespace motorized_shoe;

namespace {

struct Args {
    std::string iface = "can0";
    int left_node = 127;
    int right_node = 126;
    bool enable_left = true;
    bool enable_right = true;
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << '\n';
                std::exit(2);
            }
            return argv[++i];
        };
        if (k == "--iface") {
            a.iface = next("--iface");
        } else if (k == "--left") {
            a.left_node = std::stoi(next("--left"));
        } else if (k == "--right") {
            a.right_node = std::stoi(next("--right"));
        } else if (k == "--side") {
            std::string s = next("--side");
            if (s == "both") {
                a.enable_left = true;
                a.enable_right = true;
            } else if (s == "left") {
                a.enable_left = true;
                a.enable_right = false;
            } else if (s == "right") {
                a.enable_left = false;
                a.enable_right = true;
            } else {
                std::cerr << "--side must be one of: both, left, right (got '"
                          << s << "')\n";
                std::exit(2);
            }
        } else if (k == "-h" || k == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [--iface can0] [--left 127] [--right 126]"
                         " [--side both|left|right]\n";
            std::exit(0);
        } else {
            std::cerr << "unknown arg: " << k << '\n';
            std::exit(2);
        }
    }
    if (!a.enable_left && !a.enable_right) {
        std::cerr << "--side cannot disable both sides\n";
        std::exit(2);
    }
    return a;
}

void disable_drive(CANSocket& sock, int node_id) {
    auto nmt = create_nmt_message(static_cast<uint32_t>(node_id), CANOPEN_NMT_START);
    sock.send_message(nmt.can_id, nmt.data, nmt.dlc);

    auto vel = create_sdo_download(
        static_cast<uint32_t>(node_id), CANOPEN_TARGET_VELOCITY, 0, 0, 4);
    sock.send_message(vel.can_id, vel.data, vel.dlc);

    auto shutdown = create_sdo_download(
        static_cast<uint32_t>(node_id), CANOPEN_CONTROL_WORD, 0, CANOPEN_SHUTDOWN_STATE, 2);
    sock.send_message(shutdown.can_id, shutdown.data, shutdown.dlc);
}

}  // namespace

int main(int argc, char** argv) {
    Args args = parse_args(argc, argv);

    std::cout << "[disable] iface=" << args.iface
              << " left=" << (args.enable_left ? std::to_string(args.left_node) : std::string("off"))
              << " right=" << (args.enable_right ? std::to_string(args.right_node) : std::string("off"))
              << '\n';

    CANSocket sock(args.iface);

    bool left_ok = false;
    bool right_ok = false;

    if (args.enable_left) {
        try {
            disable_drive(sock, args.left_node);
            left_ok = true;
        } catch (const std::exception& e) {
            std::cerr << "[disable] left send failed: " << e.what() << '\n';
        }
    }
    if (args.enable_right) {
        try {
            disable_drive(sock, args.right_node);
            right_ok = true;
        } catch (const std::exception& e) {
            std::cerr << "[disable] right send failed: " << e.what() << '\n';
        }
    }

    if (!left_ok && !right_ok) {
        std::cerr << "[disable] no drives disabled\n";
        return 1;
    }

    std::cout << "[disable] done\n";
    return 0;
}
