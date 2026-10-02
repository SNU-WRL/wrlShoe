#ifndef MOTORIZED_SHOE_SLIP_SCHEDULE_HPP
#define MOTORIZED_SHOE_SLIP_SCHEDULE_HPP

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/slip_perturbation_node.hpp"

namespace motorized_shoe {

// Randomized slip schedule, read from the CSV that tools/slip_schedule_planner.html
// exports, and the runner that delivers it during one trial
// (motorized_shoe_schedule_app).
//
// Schedule CSV: '#' lines are comments (the planner writes its seed and
// settings there) and are copied into the run's records. Columns are found by
// name; the ones used are
//   slip, trial, arm_in_trial_s, foot (L/R/Left/Right),
//   type (Early|ES|HS, Mid|MS|FF, Late|LS|HO), value (peak speed, m/s),
//   unit (must contain "m/s"), and optionally magnitude (label; its trailing
//   number is used if value is blank), block, direction (anterior = +velocity,
//   posterior = -velocity; the YAML's older backward / forward are accepted
//   as the same), delay_ms (after the trigger event) and trial_duration_s.
// Blank direction / delay_ms = the YAML default for that slip type.

using SlipMode = SlipPerturbationNode::Mode;
using SlipRequest = SlipPerturbationNode::Request;
using SlipOutcome = SlipPerturbationNode::Outcome;

struct ScheduledSlip {
    int slip = 0;            // planner's slip number (unique in the file)
    int trial = 0;
    int block = 0;           // 0 = not in the file
    double arm_s = 0.0;      // planned arm time, seconds into the trial
    std::string foot;        // "Left" / "Right"
    std::string type;        // as written in the file
    SlipMode mode = SlipMode::None;
    std::string magnitude;   // label as written
    double speed_m_s = 0.0;
    std::string direction;   // "anterior" (+velocity) / "posterior" (-velocity)
    bool direction_from_file = false;
    int delay_ms = 0;
    bool delay_from_file = false;
    int duration_ms = 0;
    int32_t velocity = 0;    // signed counts/s actually commanded
    double ramp_ms = 0.0;    // time to reach |velocity| at the slip profile acceleration
};

struct SlipSchedule {
    std::string path;
    int trial = 0;
    double trial_duration_s = 0.0;
    bool trial_duration_from_file = false;
    double counts_per_m = 0.0;   // 0 = conversion not configured (dry run only)
    std::vector<ScheduledSlip> slips;   // this trial only, in arm order
    std::vector<std::string> comments;  // '#' lines of the file
    std::vector<std::string> warnings;
};

// Reads the slips of `trial`. Throws std::runtime_error listing every problem
// found. With require_conversion false (dry run) a missing m/s -> counts/s
// conversion is a warning and velocities are 0.
SlipSchedule load_slip_schedule(const std::string& path, int trial, const Config& cfg,
                                bool require_conversion);

void print_slip_schedule(std::ostream& os, const SlipSchedule& schedule);

const char* slip_mode_label(SlipMode mode);   // "early (HS)" / "mid (FF)" / "late (HO)"

// What the runner needs from a slip node; SlipPerturbationNode in the app, a
// simulated one in tools/slip_schedule_sim.
class SlipDeliverer {
public:
    virtual ~SlipDeliverer() = default;
    virtual void request(const SlipRequest& request) = 0;
    virtual bool cancel(const std::string& reason) = 0;
    virtual std::vector<SlipOutcome> take_outcomes() = 0;
};

// Delivers one trial's slips in order against a trial clock.
//
//   * The clock starts at start() and stops while paused (e-stop), so a pause
//     neither eats a slip's retry window nor shifts the schedule into it.
//   * A slip becomes due at max(planned arm, previous delivered onset +
//     min_gap_s): the gap is enforced between onsets that really happened.
//   * Not delivered (stance ended first, IMU stale, drive fault, e-stop
//     before the burst) -> re-armed until retry_window_s after it became due,
//     then recorded as missed. A refusal waits refused_retry_s before the
//     next attempt.
//   * A burst cut short by a fault / e-stop is recorded as aborted and not
//     repeated: the subject already felt it.
//   * At trial_duration_s the trial ends; a slip still waiting is cancelled
//     (missed), one already firing is allowed to finish.
//
// Every step is written as a row of the events CSV.
class SlipScheduleRunner {
public:
    struct Params {
        double min_gap_s = 10.0;
        double retry_window_s = 5.0;
        double refused_retry_s = 0.25;
    };

    SlipScheduleRunner(const SlipSchedule& schedule, const Params& params, SlipDeliverer* left,
                       SlipDeliverer* right, std::ostream& events, std::ostream& console);

    // Resume after an interrupted run: slips numbered below `slip` are recorded
    // as skipped and the clock starts at that slip's arm time minus min_gap_s
    // (not before 0). Call before start().
    void start_from_slip(int slip);
    void start(int64_t now_ns, int64_t log_origin_ns);
    void tick(int64_t now_ns, bool paused);
    // Operator quit: cancels what is waiting, records the rest as not run.
    void stop(int64_t now_ns, const std::string& reason);

    bool started() const { return started_; }
    bool finished() const { return finished_; }
    double trial_time_s() const { return started_ ? last_trial_s_ : -1.0; }
    int active_slip() const;   // slip number being delivered, 0 = none
    double start_offset_s() const { return clock_offset_s_; }
    void print_summary(std::ostream& os) const;

private:
    enum class Status {
        Pending, Requested, Armed, Triggered, Slipping, WaitRetry,
        Delivered, Missed, Aborted, Skipped, NotRun,
    };
    struct Record {
        Status status = Status::Pending;
        int attempts = 0;
        double due_s = -1.0;
        double next_attempt_s = 0.0;
        double onset_s = -1.0;
        std::string last_reason;
    };

    static const char* status_name(Status st);
    static bool is_final(Status st);
    double trial_s(int64_t ns) const;
    SlipDeliverer* deliverer_for(const ScheduledSlip& s) const;
    void handle_outcome(const SlipOutcome& o);
    void send_request(size_t i, double t);
    void finish_current(Status st, double t, const std::string& event, const std::string& reason);
    void end_trial(double t, const std::string& reason);
    void write(double t, int64_t ns, const ScheduledSlip* s, int attempt, const std::string& event,
               const std::string& detail);
    void say(double t, const std::string& text);
    std::string describe(const ScheduledSlip& s) const;

    SlipSchedule schedule_;
    Params params_;
    SlipDeliverer* left_;
    SlipDeliverer* right_;
    std::ostream& events_;
    std::ostream& console_;
    std::vector<Record> records_;
    size_t current_ = 0;           // index of the slip being handled
    bool started_ = false;
    bool finished_ = false;
    bool paused_ = false;
    int64_t start_ns_ = 0;
    int64_t log_origin_ns_ = 0;
    int64_t paused_total_ns_ = 0;
    int64_t pause_start_ns_ = 0;
    double clock_offset_s_ = 0.0;
    double last_trial_s_ = 0.0;
    double last_onset_s_ = -1e9;
    int64_t last_ns_ = 0;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_SLIP_SCHEDULE_HPP
