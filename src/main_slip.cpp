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
#include "motorized_shoe/keyboard_input.hpp"
#include "motorized_shoe/read_can_interpret_imu_node.hpp"
#include "motorized_shoe/read_can_malfunction_from_elmo_node.hpp"
#include "motorized_shoe/realtime_utils.hpp"
#include "motorized_shoe/send_can_command_to_elmo_node.hpp"
#include "motorized_shoe/slip_perturbation_node.hpp"

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
    default_log_name << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_slip_log.csv";
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

        motorized_shoe::SendCanCommandToElmoNode cmd_node(cfg, bus);
        // Slip experiment: motors must stay at 0 outside the slip burst.
        // Suppress gait-phase velocity commands so process_foot never spins
        // the motor on its own — only inject_velocity (slip burst / post-slip
        // hold) drives the motor.
        cmd_node.set_suppress_gait_velocity_commands(true);
        cmd_node.set_slip_profile_acceleration(cfg.slip.slip_profile_acceleration);
        motorized_shoe::ReadCanMalfunctionFromElmoNode status_node(cfg, bus);
        motorized_shoe::ReadCanInterpretImuNode imu_node(cfg, bus);
        motorized_shoe::GaitPhaseDetectionNode gait_node(cfg, bus);
        motorized_shoe::SlipPerturbationNode slip_node(cfg.slip, bus, cmd_node);

        motorized_shoe::KeyboardInput keyboard;
        keyboard.start([&slip_node, &cmd_node, &cfg](char c) {
            if (c == cfg.slip.mode1_key) {
                slip_node.request_slip(motorized_shoe::SlipPerturbationNode::Mode::AfterHS);
            } else if (c == cfg.slip.mode2_key) {
                slip_node.request_slip(motorized_shoe::SlipPerturbationNode::Mode::BeforeTO);
            } else if (c == 's' || c == 'S') {
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

        std::cout << "Starting slip-perturbation loop at " << loop_hz
                  << " Hz. Logging every 10 ms to " << log_path << '\n';
        std::cout << "Slip config: foot=" << cfg.slip.foot
                  << " velocity=" << cfg.slip.slip_velocity
                  << " duration=" << cfg.slip.slip_duration_ms << "ms"
                  << " hs_delay=" << cfg.slip.mode1_delay_after_hs_ms << "ms"
                  << " ho_delay=" << cfg.slip.mode2_delay_after_ho_ms << "ms\n";
        std::cout << "Keys: '" << cfg.slip.mode1_key << "' = slip after HS, '"
                  << cfg.slip.mode2_key << "' = slip on HO (before TO),"
                  << " 's' = stop motors, 'r' = resume, 'q' = quit\n";
        std::cout.flush();

        auto next_tick = std::chrono::steady_clock::now();
        while (g_run.load()) {
            next_tick += period;

            imu_node.tick();
            status_node.tick();
            gait_node.tick();
            slip_node.tick();
            cmd_node.tick();

            auto snapshot = bus.snapshot();
            logger.queue_snapshot(snapshot);

            if ((tick_count % 10) == 0) {
                logger.flush();
            }
            ++tick_count;

            std::this_thread::sleep_until(next_tick);

            const auto now = std::chrono::steady_clock::now();
            if (now > next_tick + period) {
                next_tick = now;
            }
        }

        keyboard.stop();
        logger.flush();
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
