// Offline check of the slip-schedule runner (no CAN, no hardware). Runs one
// trial of a schedule against simulated slip nodes that refuse, lose the
// stance, never see a trigger, or get e-stopped at random, then checks:
//   * every slip ends delivered / missed / aborted / not run,
//   * fired slips are at least min_gap_s apart,
//   * no slip is requested from a node that is still busy,
//   * nothing is requested while paused, nothing fires after the trial ends.
//
//   slip_schedule_sim <config.yaml> <schedule.csv> <trial> [--seed N]
//       [--p-refuse P] [--p-cancel P] [--p-stand P] [--stride S]
//       [--pause-at S --pause-for S] [--from-slip K] [--events out.csv]

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/slip_schedule.hpp"

using namespace motorized_shoe;

namespace {

struct SimParams {
    double p_refuse = 0.05;   // request refused (stale IMU, drive busy)
    double p_cancel = 0.10;   // mid/late stance ends before the fire time
    double p_stand = 0.03;    // armed, but no trigger event ever comes
    double stride_s = 1.3;
};

class SimNode : public SlipDeliverer {
public:
    SimNode(std::string foot, const SimParams& p, std::mt19937& rng) : foot_(std::move(foot)), p_(p), rng_(rng) {}

    void request(const SlipRequest& r) override {
        if (pending_ || state_ != Idle) ++busy_requests;
        if (estop_) ++requests_while_paused;
        pending_ = true;
        req_ = r;
    }
    bool cancel(const std::string& reason) override {
        if (pending_) {
            pending_ = false;
            push(req_.id, SlipOutcome::Kind::Cancelled, reason);
        }
        if (state_ == Slipping) return false;
        if (state_ != Idle) {
            push(cur_.id, SlipOutcome::Kind::Cancelled, reason);
            state_ = Idle;
        }
        return true;
    }
    std::vector<SlipOutcome> take_outcomes() override {
        std::vector<SlipOutcome> out;
        out.swap(out_);
        return out;
    }

    // The app reads one live e-stop flag in both the runner and the node, so
    // set it before the runner's tick, not after.
    void set_estop(bool estop) { estop_ = estop; }

    void tick(int64_t now, bool estop) {
        now_ = now;
        estop_ = estop;
        if (estop) {
            if (pending_) push(req_.id, SlipOutcome::Kind::Refused, "emergency stop");
            pending_ = false;
            if (state_ != Idle) {
                push(cur_.id, SlipOutcome::Kind::Aborted, "emergency stop", 0, state_ == Slipping);
                state_ = Idle;
            }
            return;
        }
        if (pending_) {
            pending_ = false;
            if (u() < p_.p_refuse) {
                push(req_.id, SlipOutcome::Kind::Refused, "IMU stale");
            } else {
                cur_ = req_;
                state_ = Armed;
                push(cur_.id, SlipOutcome::Kind::Armed, "");
                never_ = u() < p_.p_stand;
                trigger_at_ = now + static_cast<int64_t>(u() * p_.stride_s * 1e9);
            }
        }
        if (state_ == Armed && !never_ && now >= trigger_at_) {
            if (cur_.mode != SlipMode::AfterHS && u() < p_.p_cancel) {
                push(cur_.id, SlipOutcome::Kind::Cancelled, "toe-off before the fire time");
                state_ = Idle;
            } else {
                push(cur_.id, SlipOutcome::Kind::Triggered, "HS", now);
                fire_at_ = now + cur_.delay_ms * 1000000LL;
                state_ = Delaying;
            }
        }
        if (state_ == Delaying && now >= fire_at_) {
            push(cur_.id, SlipOutcome::Kind::Started, "");
            starts.push_back(now);
            end_at_ = now + cur_.duration_ms * 1000000LL;
            state_ = Slipping;
        }
        if (state_ == Slipping && now >= end_at_) {
            push(cur_.id, SlipOutcome::Kind::Completed, "");
            state_ = Idle;
        }
    }

    int busy_requests = 0;
    int requests_while_paused = 0;
    std::vector<int64_t> starts;

private:
    enum State { Idle, Armed, Delaying, Slipping };
    double u() { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_); }
    void push(int id, SlipOutcome::Kind k, const std::string& detail, int64_t ev = 0, bool during = false) {
        SlipOutcome o;
        o.id = id;
        o.kind = k;
        o.time_ns = now_;
        o.event_ns = ev;
        o.during_slip = during;
        o.detail = detail;
        out_.push_back(o);
    }

    std::string foot_;
    SimParams p_;
    std::mt19937& rng_;
    State state_ = Idle;
    bool pending_ = false;
    bool never_ = false;
    bool estop_ = false;
    SlipRequest req_;
    SlipRequest cur_;
    int64_t now_ = 0, trigger_at_ = 0, fire_at_ = 0, end_at_ = 0;
    std::vector<SlipOutcome> out_;
};

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "usage: slip_schedule_sim <config.yaml> <schedule.csv> <trial> [--seed N] [--p-refuse P]"
                     " [--p-cancel P] [--p-stand P] [--stride S] [--pause-at S --pause-for S] [--from-slip K] [--events out.csv]\n";
        return 2;
    }
    SimParams sp;
    unsigned seed = 1;
    double pause_at = -1.0, pause_for = 0.0;
    std::string events_path;
    int from_slip = 0;
    for (int i = 4; i + 1 < argc; i += 2) {
        const std::string a = argv[i];
        const std::string v = argv[i + 1];
        if (a == "--seed") seed = static_cast<unsigned>(std::stoul(v));
        else if (a == "--p-refuse") sp.p_refuse = std::stod(v);
        else if (a == "--p-cancel") sp.p_cancel = std::stod(v);
        else if (a == "--p-stand") sp.p_stand = std::stod(v);
        else if (a == "--stride") sp.stride_s = std::stod(v);
        else if (a == "--pause-at") pause_at = std::stod(v);
        else if (a == "--pause-for") pause_for = std::stod(v);
        else if (a == "--events") events_path = v;
        else if (a == "--from-slip") from_slip = std::stoi(v);
        else { std::cerr << "unknown option " << a << '\n'; return 2; }
    }

    try {
        Config cfg = load_config(argv[1]);
        if (cfg.slip_schedule.counts_per_m() <= 0) cfg.slip_schedule.counts_per_m_override = 100000.0f;  // sim only
        const SlipSchedule sched = load_slip_schedule(argv[2], std::stoi(argv[3]), cfg, true);

        std::mt19937 rng(seed);
        SimNode left("Left", sp, rng), right("Right", sp, rng);
        std::ofstream events_file;
        std::ofstream null_out;
        if (!events_path.empty()) events_file.open(events_path);
        std::ostream& events = events_path.empty() ? static_cast<std::ostream&>(null_out) : events_file;
        std::ostringstream console;

        SlipScheduleRunner::Params p;
        p.min_gap_s = cfg.slip_schedule.min_gap_s;
        p.retry_window_s = cfg.slip_schedule.retry_window_s;
        p.refused_retry_s = cfg.slip_schedule.refused_retry_ms / 1000.0;
        SlipScheduleRunner runner(sched, p, &left, &right, events, console);

        const int64_t t0 = 1000000000LL;
        const int64_t step = 1000000LL;  // 1 ms, the control loop period
        int64_t now = t0;
        if (from_slip > 0) runner.start_from_slip(from_slip);
        runner.start(now, t0);
        int64_t paused_ns = 0;
        int64_t last_trial_end_ns = 0;
        int fired_after_end = 0;
        const int64_t limit = t0 + static_cast<int64_t>((sched.trial_duration_s + 3600) * 1e9);
        while (!runner.finished() && now < limit) {
            now += step;
            const double wall = (now - t0) / 1e9;
            const bool paused = pause_at >= 0 && wall >= pause_at && wall < pause_at + pause_for;
            if (paused) paused_ns += step;
            left.set_estop(paused);
            right.set_estop(paused);
            runner.tick(now, paused);
            const size_t before = left.starts.size() + right.starts.size();
            left.tick(now, paused);
            right.tick(now, paused);
            const double trial_t = runner.start_offset_s() + (now - t0 - paused_ns) / 1e9;
            if (left.starts.size() + right.starts.size() > before && trial_t > sched.trial_duration_s) {
                ++fired_after_end;
            }
            last_trial_end_ns = now;
        }
        (void)last_trial_end_ns;

        std::vector<int64_t> starts = left.starts;
        starts.insert(starts.end(), right.starts.begin(), right.starts.end());
        std::sort(starts.begin(), starts.end());
        double min_gap = 1e9;
        for (size_t i = 1; i < starts.size(); ++i) min_gap = std::min(min_gap, (starts[i] - starts[i - 1]) / 1e9);

        runner.print_summary(std::cout);
        // A pause stops the trial clock, so a wall-clock gap spanning it is longer, never shorter.
        const bool gap_ok = starts.size() < 2 || min_gap >= p.min_gap_s - 1e-3;
        const bool busy_ok = left.busy_requests == 0 && right.busy_requests == 0;
        const bool end_ok = fired_after_end == 0;
        const bool done_ok = runner.finished();
        const int paused_req = left.requests_while_paused + right.requests_while_paused;
        const bool pause_ok = paused_req == 0;
        std::cout << "sim seed " << seed << ": " << starts.size() << " fired, min gap "
                  << (starts.size() < 2 ? 0.0 : min_gap) << " s [" << (gap_ok ? "ok" : "FAIL")
                  << "], busy requests " << left.busy_requests + right.busy_requests << " [" << (busy_ok ? "ok" : "FAIL")
                  << "], fired after trial end " << fired_after_end << " [" << (end_ok ? "ok" : "FAIL")
                  << "], requests while paused " << paused_req << " [" << (pause_ok ? "ok" : "FAIL")
                  << "], finished [" << (done_ok ? "ok" : "FAIL") << "]\n";
        return (gap_ok && busy_ok && end_ok && done_ok && pause_ok) ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    }
}
