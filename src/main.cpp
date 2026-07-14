#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <filesystem>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"
#include "motorized_shoe/data_logger.hpp"
#include "motorized_shoe/gait_phase_detection_node.hpp"
#include "motorized_shoe/keyboard_input.hpp"
#include "motorized_shoe/read_can_interpret_imu_node.hpp"
#include "motorized_shoe/read_can_malfunction_from_elmo_node.hpp"
#include "motorized_shoe/realtime_utils.hpp"
#include "motorized_shoe/send_can_command_to_elmo_node.hpp"

namespace {
std::atomic<bool> g_run{true};

void signal_handler(int signal_number) {
    (void)signal_number;
    g_run = false;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string config_path;
    std::ostringstream ts_stream;
    const auto now_sys = std::chrono::system_clock::now();
    const std::time_t now_t = std::chrono::system_clock::to_time_t(now_sys);
    std::tm tm_now{};
    localtime_r(&now_t, &tm_now);
    ts_stream << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    const std::string timestamp = ts_stream.str();
    // Empty => auto-named after config load as
    // <timestamp>_tec_v<Swing velocity>_a<profile_acceleration>_log.csv
    std::string log_path;

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

        if (log_path.empty()) {
            const auto swing_it = cfg.velocity_map.find("Swing");
            const int32_t swing_velocity =
                (swing_it != cfg.velocity_map.end()) ? swing_it->second : 0;
            std::ostringstream name;
            name << timestamp << "_tec_v" << swing_velocity
                 << "_a" << cfg.profile_acceleration << "_log.csv";
            log_path = name.str();
        }

        motorized_shoe::DataBus bus;
        motorized_shoe::DataLogger logger(log_path);

        // cmd_node is constructed first so its background ELMO init starts
        // before the IMU socket on can1 is opened. This avoids piling up
        // can1 frames in the kernel queue while the shared SPI bus is busy
        // with the can0 init burst.
        motorized_shoe::SendCanCommandToElmoNode cmd_node(cfg, bus);
        motorized_shoe::ReadCanMalfunctionFromElmoNode status_node(cfg, bus);
        motorized_shoe::ReadCanInterpretImuNode imu_node(cfg, bus);
        motorized_shoe::GaitPhaseDetectionNode gait_node(cfg, bus);

        motorized_shoe::KeyboardInput keyboard;
        keyboard.start([&cmd_node](char c) {
            if (c == 's' || c == 'S') {
                cmd_node.request_emergency_stop(true);
            } else if (c == 'r' || c == 'R') {
                cmd_node.request_emergency_stop(false);
            } else if (c == 'q' || c == 'Q') {
                g_run.store(false);
            }
        });

        const int loop_hz = (cfg.loop_frequency_hz > 0) ? cfg.loop_frequency_hz : 1000;
        const auto period = std::chrono::microseconds(1000000 / loop_hz);
        uint64_t tick_count = 0;
        uint32_t prev_log_us = 0;  // previous tick's logging cost (see log_latency_us)

        struct LatencyStats {
            uint64_t samples = 0;
            uint64_t imu_sum_us = 0;
            uint64_t status_sum_us = 0;
            uint64_t gait_sum_us = 0;
            uint64_t cmd_sum_us = 0;
            uint64_t loop_sum_us = 0;
            uint32_t imu_max_us = 0;
            uint32_t status_max_us = 0;
            uint32_t gait_max_us = 0;
            uint32_t cmd_max_us = 0;
            uint32_t loop_max_us = 0;
        } stats;

        std::cout << "Starting loop at " << loop_hz << " Hz. Logging every 10ms to " << log_path << '\n';
        std::cout << "Keys: 's' = stop motors, 'r' = resume, 'q' = quit\n";
        std::cout.flush();

        auto next_tick = std::chrono::steady_clock::now();
        while (g_run.load()) {
            next_tick += period;

            const auto tick_start = std::chrono::steady_clock::now();

            const auto imu_start = std::chrono::steady_clock::now();
            imu_node.tick();
            const auto imu_end = std::chrono::steady_clock::now();

            const auto status_start = imu_end;
            status_node.tick();
            const auto status_end = std::chrono::steady_clock::now();

            const auto gait_start = status_end;
            gait_node.tick();
            const auto gait_end = std::chrono::steady_clock::now();

            const auto cmd_start = gait_end;
            cmd_node.tick();
            const auto cmd_end = std::chrono::steady_clock::now();

            const uint32_t imu_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(imu_end - imu_start).count());
            const uint32_t status_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(status_end - status_start).count());
            const uint32_t gait_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(gait_end - gait_start).count());
            const uint32_t cmd_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(cmd_end - cmd_start).count());
            const uint32_t loop_us = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(cmd_end - tick_start).count());

            ++stats.samples;
            stats.imu_sum_us += imu_us;
            stats.status_sum_us += status_us;
            stats.gait_sum_us += gait_us;
            stats.cmd_sum_us += cmd_us;
            stats.loop_sum_us += loop_us;

            if (imu_us > stats.imu_max_us) stats.imu_max_us = imu_us;
            if (status_us > stats.status_max_us) stats.status_max_us = status_us;
            if (gait_us > stats.gait_max_us) stats.gait_max_us = gait_us;
            if (cmd_us > stats.cmd_max_us) stats.cmd_max_us = cmd_us;
            if (loop_us > stats.loop_max_us) stats.loop_max_us = loop_us;

            // Time the logging path (snapshot copy + queue + every-10th-tick
            // flush) separately from loop_latency_us. The cost is carried into
            // the next snapshot, so log_latency_us is one tick delayed.
            const auto log_start = std::chrono::steady_clock::now();
            auto snapshot = bus.snapshot();
            snapshot.imu_node_latency_us = imu_us;
            snapshot.status_node_latency_us = status_us;
            snapshot.gait_node_latency_us = gait_us;
            snapshot.command_node_latency_us = cmd_us;
            snapshot.loop_latency_us = loop_us;
            snapshot.log_latency_us = prev_log_us;
            logger.queue_snapshot(snapshot);

            if ((tick_count % 10) == 0) {
                logger.flush();
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

        keyboard.stop();
        logger.flush();

        if (stats.samples > 0) {
            auto avg = [samples = stats.samples](uint64_t sum) {
                return static_cast<double>(sum) / static_cast<double>(samples);
            };

            std::cout << "Latency summary (us): "
                      << "imu avg=" << avg(stats.imu_sum_us) << " max=" << stats.imu_max_us
                      << ", status avg=" << avg(stats.status_sum_us) << " max=" << stats.status_max_us
                      << ", gait avg=" << avg(stats.gait_sum_us) << " max=" << stats.gait_max_us
                      << ", command avg=" << avg(stats.cmd_sum_us) << " max=" << stats.cmd_max_us
                      << ", loop avg=" << avg(stats.loop_sum_us) << " max=" << stats.loop_max_us
                      << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
