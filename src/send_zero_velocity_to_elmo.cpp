// Continuously commands target velocity = 0 to both ELMO drives via SocketCAN.
//
// Usage:
//   sudo ./build/send_zero_velocity_to_elmo [--iface can0]
//        [--left 127] [--right 126] [--rate-hz 100] [--log path.csv]
//        [--side both|left|right]
//
// Press Ctrl+C to stop. On exit the program tries to leave the drives in a
// safe state (one final velocity-zero SDO).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "motorized_shoe/can_utils.hpp"
#include "motorized_shoe/canopen_utils.hpp"
#include "motorized_shoe/realtime_utils.hpp"

using namespace motorized_shoe;

namespace {

std::atomic<bool> g_running{true};

void handle_signal(int /*signo*/) {
    g_running.store(false, std::memory_order_relaxed);
}

void send_velocity_zero(CANSocket& sock, int node_id) {
    auto msg = create_sdo_download(
        static_cast<uint32_t>(node_id), CANOPEN_TARGET_VELOCITY, 0, 0, 4);
    sock.send_message(msg.can_id, msg.data, msg.dlc);
}

void initialize_elmo(CANSocket& sock, int node_id, int32_t accel_value) {
    using namespace std::chrono_literals;

    auto nmt = create_nmt_message(static_cast<uint32_t>(node_id), CANOPEN_NMT_START);
    sock.send_message(nmt.can_id, nmt.data, nmt.dlc);
    std::this_thread::sleep_for(100ms);

    auto fault_reset = create_sdo_download(
        node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_FAULT_RESET, 2);
    sock.send_message(fault_reset.can_id, fault_reset.data, fault_reset.dlc);
    std::this_thread::sleep_for(50ms);

    // Mode of operation = 3 (Profile Velocity Mode)
    auto mode = create_sdo_download(node_id, CANOPEN_MODE_OF_OPERATION, 0, 3, 1);
    sock.send_message(mode.can_id, mode.data, mode.dlc);
    std::this_thread::sleep_for(50ms);

    // Profile accel/decel. Was hardcoded to 1e6, which silently knocked the
    // drive's ramp back to 1e6 whenever this tool ran between slip sessions.
    // Now sourced from --accel (default matches the app's 1e7) so it can't
    // reset the drive behind the main app's back.
    const uint32_t accel = static_cast<uint32_t>(accel_value);
    auto accel_msg = create_sdo_download(node_id, 0x6083, 0, accel, 4);
    sock.send_message(accel_msg.can_id, accel_msg.data, accel_msg.dlc);
    std::this_thread::sleep_for(50ms);

    auto decel_msg = create_sdo_download(node_id, 0x6084, 0, accel, 4);
    sock.send_message(decel_msg.can_id, decel_msg.data, decel_msg.dlc);
    std::this_thread::sleep_for(50ms);

    auto shutdown = create_sdo_download(
        node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SHUTDOWN_STATE, 2);
    sock.send_message(shutdown.can_id, shutdown.data, shutdown.dlc);
    std::this_thread::sleep_for(50ms);

    auto switch_on = create_sdo_download(
        node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_SWITCH_ON_STATE, 2);
    sock.send_message(switch_on.can_id, switch_on.data, switch_on.dlc);
    std::this_thread::sleep_for(50ms);

    auto enable = create_sdo_download(
        node_id, CANOPEN_CONTROL_WORD, 0, CANOPEN_ENABLE_OPERATION_STATE, 2);
    sock.send_message(enable.can_id, enable.data, enable.dlc);
    std::this_thread::sleep_for(50ms);
}

struct Args {
    std::string iface = "can0";
    int left_node = 127;
    int right_node = 126;
    double rate_hz = 100.0;
    std::string log_path;  // empty => auto-named timestamped file
    bool enable_left = true;
    bool enable_right = true;
    int32_t accel = 10000000;  // 0x6083/0x6084 written at init; matches the app default
};

// Velocity is always 0 in this tool, so only the accel config is embedded:
// <timestamp>_zero_a<accel>_log.csv
std::string default_log_name(int32_t accel) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_zero_a" << accel << "_log.csv";
    return oss.str();
}

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
        } else if (k == "--rate-hz") {
            a.rate_hz = std::stod(next("--rate-hz"));
        } else if (k == "--accel") {
            a.accel = static_cast<int32_t>(std::stol(next("--accel")));
        } else if (k == "--log") {
            a.log_path = next("--log");
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
                         " [--rate-hz 100] [--log path.csv]"
                         " [--side both|left|right] [--accel 10000000]\n";
            std::exit(0);
        } else {
            std::cerr << "unknown arg: " << k << '\n';
            std::exit(2);
        }
    }
    if (a.rate_hz <= 0.0) {
        std::cerr << "--rate-hz must be > 0\n";
        std::exit(2);
    }
    if (!a.enable_left && !a.enable_right) {
        std::cerr << "--side cannot disable both sides\n";
        std::exit(2);
    }
    if (a.log_path.empty()) {
        a.log_path = default_log_name(a.accel);
    }
    return a;
}

}  // namespace

int main(int argc, char** argv) {
    Args args = parse_args(argc, argv);

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    motorized_shoe::set_realtime_priority(95, "zero-vel");

    std::cout << "[zero-vel] iface=" << args.iface
              << " left=" << (args.enable_left ? std::to_string(args.left_node) : std::string("off"))
              << " right=" << (args.enable_right ? std::to_string(args.right_node) : std::string("off"))
              << " rate=" << args.rate_hz << " Hz"
              << " accel=" << args.accel
              << " log=" << args.log_path << '\n';

    std::ofstream log(args.log_path);
    if (!log.is_open()) {
        std::cerr << "[zero-vel] failed to open log file: " << args.log_path << '\n';
        return 1;
    }
    log << "t_s,tick,left_node,right_node,target_velocity,"
           "left_send_ok,right_send_ok,left_send_errors,right_send_errors\n";

    CANSocket sock(args.iface);

    bool left_ok = args.enable_left;
    bool right_ok = args.enable_right;
    if (left_ok) {
        try {
            initialize_elmo(sock, args.left_node, args.accel);
        } catch (const std::exception& e) {
            std::cerr << "[zero-vel] left init failed: " << e.what() << '\n';
            left_ok = false;
        }
    }
    if (right_ok) {
        try {
            initialize_elmo(sock, args.right_node, args.accel);
        } catch (const std::exception& e) {
            std::cerr << "[zero-vel] right init failed: " << e.what() << '\n';
            right_ok = false;
        }
    }
    if (!left_ok && !right_ok) {
        std::cerr << "[zero-vel] no drives available, aborting\n";
        return 1;
    }

    const auto period = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / args.rate_hz));
    const auto t0 = std::chrono::steady_clock::now();
    auto next_tick = t0;

    uint64_t sent = 0;
    uint64_t left_send_errors = 0;
    uint64_t right_send_errors = 0;
    const uint64_t flush_every = static_cast<uint64_t>(args.rate_hz);  // ~1 s

    while (g_running.load(std::memory_order_relaxed)) {
        bool left_send_ok = false;
        bool right_send_ok = false;

        if (left_ok) {
            try {
                send_velocity_zero(sock, args.left_node);
                left_send_ok = true;
            } catch (const std::exception& e) {
                ++left_send_errors;
                std::cerr << "[zero-vel] left send failed: " << e.what() << '\n';
            }
        }
        if (right_ok) {
            try {
                send_velocity_zero(sock, args.right_node);
                right_send_ok = true;
            } catch (const std::exception& e) {
                ++right_send_errors;
                std::cerr << "[zero-vel] right send failed: " << e.what() << '\n';
            }
        }
        ++sent;

        const double t_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        log << std::fixed << std::setprecision(6) << t_s << ','
            << sent << ','
            << args.left_node << ',' << args.right_node << ",0,"
            << (left_send_ok ? 1 : 0) << ','
            << (right_send_ok ? 1 : 0) << ','
            << left_send_errors << ',' << right_send_errors << '\n';

        if ((sent % flush_every) == 0) {
            log.flush();
            std::cout << "[zero-vel] sent " << sent
                      << " frames, errors L=" << left_send_errors
                      << " R=" << right_send_errors << '\n';
        }

        next_tick += period;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "[zero-vel] stopping, sending final velocity=0\n";
    try {
        if (left_ok) send_velocity_zero(sock, args.left_node);
        if (right_ok) send_velocity_zero(sock, args.right_node);
    } catch (const std::exception& e) {
        std::cerr << "[zero-vel] final send failed: " << e.what() << '\n';
    }

    log.flush();
    log.close();
    std::cout << "[zero-vel] log written to " << args.log_path << '\n';
    return 0;
}
