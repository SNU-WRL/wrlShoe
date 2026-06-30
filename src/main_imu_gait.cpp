#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"
#include "motorized_shoe/data_logger.hpp"
#include "motorized_shoe/gait_phase_detection_node.hpp"
#include "motorized_shoe/read_can_interpret_imu_node.hpp"
#include "motorized_shoe/realtime_utils.hpp"

namespace {
std::atomic<bool> g_run{true};

void signal_handler(int signal_number) {
    (void)signal_number;
    g_run = false;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string config_path;
    std::ostringstream default_log_name;
    const auto now_sys = std::chrono::system_clock::now();
    const std::time_t now_t = std::chrono::system_clock::to_time_t(now_sys);
    std::tm tm_now{};
    localtime_r(&now_t, &tm_now);
    default_log_name << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_imu_gait_log.csv";
    std::string log_path = default_log_name.str();

    const char* default_paths[] = {
        "config/motorized_shoe_params.yaml",
        "../config/motorized_shoe_params.yaml",
        "../../config/motorized_shoe_params.yaml"};

    for (const char* candidate : default_paths) {
        if (std::filesystem::exists(candidate)) {
            config_path = candidate;
            break;
        }
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    motorized_shoe::set_realtime_priority(99, "main");

    try {
        if (config_path.empty()) {
            throw std::runtime_error("Could not find default config file; pass --config <path>");
        }

        const motorized_shoe::Config cfg = motorized_shoe::load_config(config_path);

        motorized_shoe::DataBus bus;
        motorized_shoe::DataLogger logger(log_path);

        motorized_shoe::ReadCanInterpretImuNode imu_node(cfg, bus);
        motorized_shoe::GaitPhaseDetectionNode gait_node(cfg, bus);

        const int loop_hz = (cfg.loop_frequency_hz > 0) ? cfg.loop_frequency_hz : 1000;
        const auto period = std::chrono::microseconds(1000000 / loop_hz);
        uint64_t tick_count = 0;
        uint32_t prev_log_us = 0;  // previous tick's logging cost (see log_latency_us)

        std::cout << "Starting IMU+Gait loop at " << loop_hz
                  << " Hz. Logging every 10ms to " << log_path << '\n';
        std::cout.flush();

        auto next_tick = std::chrono::steady_clock::now();
        while (g_run.load()) {
            next_tick += period;

            const auto tick_start = std::chrono::steady_clock::now();

            const auto imu_start = std::chrono::steady_clock::now();
            imu_node.tick();
            const auto imu_end = std::chrono::steady_clock::now();

            const auto gait_start = imu_end;
            gait_node.tick();
            const auto gait_end = std::chrono::steady_clock::now();

            const uint32_t imu_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(imu_end - imu_start).count());
            const uint32_t gait_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(gait_end - gait_start).count());
            const uint32_t loop_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(gait_end - tick_start).count());

            // Time the logging path (snapshot copy + queue + periodic flush)
            // separately from loop_latency_us. Carried into the next snapshot,
            // so log_latency_us is one tick delayed. Includes the every-5s
            // console print in this diagnostic app.
            const auto log_start = std::chrono::steady_clock::now();
            auto snapshot = bus.snapshot();
            snapshot.imu_node_latency_us = imu_us;
            snapshot.gait_node_latency_us = gait_us;
            snapshot.loop_latency_us = loop_us;
            snapshot.log_latency_us = prev_log_us;
            logger.queue_snapshot(snapshot);

            if ((tick_count % (loop_hz / 10)) == 0) {  // Every 100ms
                logger.flush();
                if ((tick_count % (loop_hz * 5)) == 0) {  // Every 5 seconds
                    std::cout << "Running... tick=" << tick_count
                              << " imu_left_msgs=" << snapshot.imu_left.msg_count
                              << " imu_right_msgs=" << snapshot.imu_right.msg_count << '\n';
                    std::cout.flush();
                }
            }
            prev_log_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - log_start).count());
            ++tick_count;

            std::this_thread::sleep_until(next_tick);

            const auto now = std::chrono::steady_clock::now();
            if (now > next_tick + period) {
                next_tick = now;
            }
        }

        logger.flush();
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
