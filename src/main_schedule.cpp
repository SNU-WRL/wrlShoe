// Schedule-driven slip experiment: delivers one trial of a randomized slip
// schedule (the CSV exported by tools/slip_schedule_planner.html).
//
//   motorized_shoe_schedule_app --schedule <file.csv> --trial <N>
//                               [--from-slip <K>] [--config <yaml>] [--dry-run]
//
// --dry-run loads and checks the schedule, prints what would be commanded and
// exits without touching CAN. --from-slip resumes an interrupted trial at
// slip K (earlier slips are recorded as skipped).
//
// Each run writes, under one prefix <timestamp>_sched_trial<N>:
//   _log.csv       the usual 1 kHz log; sched_slip / sched_trial_time_s and
//                  slip_*_state tie each row to the schedule
//   _events.csv    one row per scheduler step (request, armed, triggered,
//                  started, delivered, refused, cancelled, missed, ...)
//   _schedule.csv  copy of the schedule file as run
//   _summary.txt   the parsed plan and delivered / missed per combination
//
// Keys: 'g' start the trial clock, 's' stop motors (pauses the trial clock),
// 'r' resume, 'q' quit. The manual slip keys are disabled here.

#include <pthread.h>
#include <sched.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"
#include "motorized_shoe/data_logger.hpp"
#include "motorized_shoe/gait_phase_detection_node.hpp"
#include "motorized_shoe/imu_watchdog.hpp"
#include "motorized_shoe/keyboard_input.hpp"
#include "motorized_shoe/read_can_interpret_imu_node.hpp"
#include "motorized_shoe/read_can_malfunction_from_elmo_node.hpp"
#include "motorized_shoe/realtime_utils.hpp"
#include "motorized_shoe/send_can_command_to_elmo_node.hpp"
#include "motorized_shoe/slip_perturbation_node.hpp"
#include "motorized_shoe/slip_schedule.hpp"

namespace {
std::atomic<bool> g_run{true};

void signal_handler(int signal_number) {
    (void)signal_number;
    g_run = false;
}

class NodeDeliverer : public motorized_shoe::SlipDeliverer {
public:
    explicit NodeDeliverer(motorized_shoe::SlipPerturbationNode& node) : node_(node) {}
    void request(const motorized_shoe::SlipRequest& r) override { node_.request_slip(r); }
    bool cancel(const std::string& reason) override { return node_.cancel(reason); }
    std::vector<motorized_shoe::SlipOutcome> take_outcomes() override { return node_.take_outcomes(); }

private:
    motorized_shoe::SlipPerturbationNode& node_;
};

// Writes the events CSV on its own normal-priority thread. The runner writes
// into a string buffer on the control thread and the loop hands it over here,
// so an SD-card stall (10-95 ms seen on 2026-09 logs) can never hold up the
// 1 kHz loop -- e.g. delay the end of a slip burst.
class AsyncFileWriter {
public:
    explicit AsyncFileWriter(const std::string& path) : file_(path) {
        if (!file_.is_open()) {
            throw std::runtime_error("cannot open " + path);
        }
        thread_ = std::thread([this] { run(); });
    }
    ~AsyncFileWriter() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_one();
        thread_.join();
    }
    void push(std::string text) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_ += text;
        }
        cv_.notify_one();
    }

private:
    void run() {
        // Created from the SCHED_FIFO control thread, so demote first.
        sched_param sp{};
        sp.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
        std::string batch;
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&] { return stop_ || !pending_.empty(); });
                if (pending_.empty() && stop_) return;
                batch.swap(pending_);
            }
            file_ << batch;
            file_.flush();
            batch.clear();
        }
    }

    std::ofstream file_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::string pending_;
    bool stop_ = false;
    std::thread thread_;
};

void usage() {
    std::cerr << "usage: motorized_shoe_schedule_app --schedule <file.csv> --trial <N>"
                 " [--from-slip <K>] [--config <yaml>] [--dry-run]\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string config_path;
    std::string schedule_path;
    int trial = 0;
    int from_slip = 0;
    bool dry_run = false;

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

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc) {
                config_path = argv[++i];
            } else if (arg == "--schedule" && i + 1 < argc) {
                schedule_path = argv[++i];
            } else if (arg == "--trial" && i + 1 < argc) {
                trial = std::stoi(argv[++i]);
            } else if (arg == "--from-slip" && i + 1 < argc) {
                from_slip = std::stoi(argv[++i]);
            } else if (arg == "--dry-run") {
                dry_run = true;
            } else {
                usage();
                return 2;
            }
        }
    } catch (const std::exception&) {
        usage();
        return 2;
    }
    if (schedule_path.empty() || trial <= 0) {
        usage();
        return 2;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        if (config_path.empty()) {
            throw std::runtime_error("Could not find default config file; pass --config <path>");
        }
        const motorized_shoe::Config cfg = motorized_shoe::load_config(config_path);
        const motorized_shoe::SlipSchedule schedule =
            motorized_shoe::load_slip_schedule(schedule_path, trial, cfg, !dry_run);
        motorized_shoe::print_slip_schedule(std::cout, schedule);

        motorized_shoe::SlipScheduleRunner::Params params;
        params.min_gap_s = cfg.slip_schedule.min_gap_s;
        params.retry_window_s = cfg.slip_schedule.retry_window_s;
        params.refused_retry_s = cfg.slip_schedule.refused_retry_ms / 1000.0;
        std::cout << "Rules: min gap " << params.min_gap_s << " s between fired slips, retry for "
                  << params.retry_window_s << " s after a slip is due, burst " << cfg.slip.slip_duration_ms
                  << " ms at profile acceleration " << cfg.slip.slip_profile_acceleration << " counts/s^2\n";

        if (dry_run) {
            std::cout << "Dry run: nothing commanded.\n";
            return 0;
        }
        if (!cfg.slip.enabled) {
            throw std::runtime_error("slip_perturbation.enabled is false in " + config_path);
        }

        std::ostringstream ts_stream;
        const std::time_t now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm_now{};
        localtime_r(&now_t, &tm_now);
        ts_stream << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_sched_trial" << trial;
        const std::string prefix = ts_stream.str();

        std::filesystem::copy_file(schedule_path, prefix + "_schedule.csv",
                                   std::filesystem::copy_options::overwrite_existing);
        motorized_shoe::set_realtime_priority(99, "main");

        AsyncFileWriter events_file(prefix + "_events.csv");
        std::ostringstream events;   // filled by the runner, drained every tick
        auto drain_events = [&] {
            if (events.tellp() > 0) {
                events_file.push(events.str());
                events.str("");
                events.clear();
            }
        };

        motorized_shoe::DataBus bus;
        motorized_shoe::DataLogger logger(prefix + "_log.csv");

        motorized_shoe::SendCanCommandToElmoNode cmd_node(cfg, bus, cfg.slip.slip_profile_acceleration);
        // As in the slip app: the motors stay at 0 except during a slip burst.
        cmd_node.set_suppress_gait_velocity_commands(true);
        motorized_shoe::ReadCanMalfunctionFromElmoNode status_node(cfg, bus);
        motorized_shoe::ReadCanInterpretImuNode imu_node(cfg, bus);
        motorized_shoe::GaitPhaseDetectionNode gait_node(cfg, bus);
        motorized_shoe::ImuWatchdog imu_watchdog(cfg.imu_stale_ms);

        // One slip node per foot the schedule uses.
        std::unique_ptr<motorized_shoe::SlipPerturbationNode> slip_left;
        std::unique_ptr<motorized_shoe::SlipPerturbationNode> slip_right;
        std::unique_ptr<NodeDeliverer> deliver_left;
        std::unique_ptr<NodeDeliverer> deliver_right;
        for (const auto& s : schedule.slips) {
            auto& node = (s.foot == "Left") ? slip_left : slip_right;
            auto& deliver = (s.foot == "Left") ? deliver_left : deliver_right;
            if (!node) {
                motorized_shoe::SlipConfig foot_cfg = cfg.slip;
                foot_cfg.foot = s.foot;
                node = std::make_unique<motorized_shoe::SlipPerturbationNode>(foot_cfg, bus, cmd_node);
                deliver = std::make_unique<NodeDeliverer>(*node);
            }
        }

        motorized_shoe::SlipScheduleRunner runner(schedule, params, deliver_left.get(), deliver_right.get(),
                                                  events, std::cout);
        if (from_slip > 0) {
            runner.start_from_slip(from_slip);
            std::cout << "Resuming at slip " << from_slip << ": trial clock starts at " << std::fixed
                      << std::setprecision(1) << runner.start_offset_s() << " s\n";
        }

        std::atomic<bool> start_requested{false};
        motorized_shoe::KeyboardInput keyboard;
        keyboard.start([&](char c) {
            if (c == 'g' || c == 'G') {
                start_requested.store(true);
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
        uint32_t prev_log_us = 0;
        int64_t log_origin_ns = 0;
        bool start_refused_reported = false;

        std::cout << "Logging to " << prefix << "_log.csv, events to " << prefix << "_events.csv\n"
                  << "Keys: 'g' = START trial " << trial << ", 's' = stop motors (pauses the trial clock),"
                  << " 'r' = resume, 'q' = quit\n";
        std::cout.flush();

        auto next_tick = std::chrono::steady_clock::now();
        while (g_run.load()) {
            next_tick += period;
            const auto tick_start = std::chrono::steady_clock::now();

            imu_node.tick();
            const auto imu_end = std::chrono::steady_clock::now();
            status_node.tick();
            const auto status_end = std::chrono::steady_clock::now();
            gait_node.tick();
            const auto gait_end = std::chrono::steady_clock::now();

            if (start_requested.load() && !runner.started() && tick_count > 0) {
                if (cmd_node.is_emergency_stopped()) {
                    if (!start_refused_reported) {
                        std::cout << "[sched] not started: emergency stop is active, press 'r' first\n";
                        start_refused_reported = true;
                    }
                    start_requested.store(false);
                } else {
                    runner.start(motorized_shoe::now_ns(), log_origin_ns);
                }
            }
            if (!start_requested.load()) {
                start_refused_reported = false;
            }
            runner.tick(motorized_shoe::now_ns(), cmd_node.is_emergency_stopped());
            drain_events();
            if (slip_left) slip_left->tick();
            if (slip_right) slip_right->tick();

            const auto cmd_start = std::chrono::steady_clock::now();
            cmd_node.tick();
            const auto cmd_end = std::chrono::steady_clock::now();

            auto us = [](auto a, auto b) {
                return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(b - a).count());
            };

            const auto log_start = std::chrono::steady_clock::now();
            auto snapshot = bus.snapshot();
            if (tick_count == 0) {
                log_origin_ns = snapshot.timestamp_ns;   // the logger's time_s = 0
            }
            imu_watchdog.check(snapshot);
            snapshot.imu_node_latency_us = us(tick_start, imu_end);
            snapshot.status_node_latency_us = us(imu_end, status_end);
            snapshot.gait_node_latency_us = us(status_end, gait_end);
            snapshot.command_node_latency_us = us(cmd_start, cmd_end);
            snapshot.loop_latency_us = us(tick_start, cmd_end);
            snapshot.log_latency_us = prev_log_us;
            snapshot.slip_left_state = slip_left ? static_cast<uint8_t>(slip_left->state()) : 0;
            snapshot.slip_right_state = slip_right ? static_cast<uint8_t>(slip_right->state()) : 0;
            snapshot.sched_slip = runner.active_slip();
            snapshot.sched_trial_time_s = runner.trial_time_s();
            logger.queue_snapshot(snapshot);

            if ((tick_count % 10) == 0) {
                logger.flush();
            }
            prev_log_us = us(log_start, std::chrono::steady_clock::now());
            ++tick_count;

            if (runner.finished()) {
                break;
            }

            std::this_thread::sleep_until(next_tick);
            const auto now = std::chrono::steady_clock::now();
            if (now > next_tick + period) {
                next_tick = now;
            }
        }

        if (!runner.finished()) {
            runner.stop(motorized_shoe::now_ns(), runner.started() ? "operator quit" : "quit before start");
        }
        drain_events();
        keyboard.stop();
        // Never leave a drive running or armed after the loop.
        cmd_node.stop_all_drives();
        logger.flush();

        runner.print_summary(std::cout);
        std::ofstream summary(prefix + "_summary.txt");
        motorized_shoe::print_slip_schedule(summary, schedule);
        runner.print_summary(summary);
        std::cout << "Wrote " << prefix << "_{log,events,schedule}.csv and " << prefix << "_summary.txt\n";
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
