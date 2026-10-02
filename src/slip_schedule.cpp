#include "motorized_shoe/slip_schedule.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace motorized_shoe {

namespace {

std::string trim(const std::string& in) {
    size_t b = 0;
    size_t e = in.size();
    while (b < e && std::isspace(static_cast<unsigned char>(in[b])) != 0) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(in[e - 1])) != 0) --e;
    return in.substr(b, e - b);
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// One CSV record; handles "quoted, fields" and "" escapes (what the planner writes).
std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quoted) {
            if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
                cur += '"';
                ++i;
            } else if (c == '"') {
                quoted = false;
            } else {
                cur += c;
            }
        } else if (c == '"') {
            quoted = true;
        } else if (c == ',') {
            out.push_back(trim(cur));
            cur.clear();
        } else {
            cur += c;
        }
    }
    out.push_back(trim(cur));
    return out;
}

bool parse_double(const std::string& s, double& out) {
    if (s.empty()) return false;
    try {
        size_t used = 0;
        out = std::stod(s, &used);
        return used == s.size() && std::isfinite(out);
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_int(const std::string& s, int& out) {
    double d = 0.0;
    if (!parse_double(s, d) || d != std::floor(d) || std::fabs(d) > 1e9) return false;
    out = static_cast<int>(d);
    return true;
}

// "Low 0.6" -> 0.6; no trailing number -> false.
bool trailing_number(const std::string& label, double& out) {
    size_t e = label.size();
    while (e > 0 && std::isspace(static_cast<unsigned char>(label[e - 1])) != 0) --e;
    size_t b = e;
    while (b > 0 && (std::isdigit(static_cast<unsigned char>(label[b - 1])) != 0 || label[b - 1] == '.')) --b;
    return b < e && parse_double(label.substr(b, e - b), out);
}

bool parse_foot(const std::string& s, std::string& out) {
    const std::string l = lower(s);
    if (l == "l" || l == "left") { out = "Left"; return true; }
    if (l == "r" || l == "right") { out = "Right"; return true; }
    return false;
}

SlipMode parse_type(const std::string& s) {
    const std::string l = lower(s);
    static const std::set<std::string> early{"early", "es", "hs", "early stance", "afterhs"};
    static const std::set<std::string> mid{"mid", "ms", "ff", "mid stance", "midstance"};
    static const std::set<std::string> late{"late", "ls", "ho", "late stance", "latestance"};
    if (early.count(l)) return SlipMode::AfterHS;
    if (mid.count(l)) return SlipMode::MidStance;
    if (late.count(l)) return SlipMode::LateStance;
    return SlipMode::None;
}

std::string fmt(double v, int prec) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
}

std::string csv_field(const std::string& s) {
    if (s.find_first_of(",\"\n") == std::string::npos) return s;
    std::string q = "\"";
    for (char c : s) {
        if (c == '"') q += '"';
        q += c;
    }
    return q + '"';
}

const char* trigger_name(SlipMode mode) {
    switch (mode) {
        case SlipMode::MidStance: return "FF";
        case SlipMode::LateStance: return "HO";
        default: return "HS";
    }
}

// "Left HS+150 anterior 0.80 m/s": the condition a slip belongs to.
std::string condition_of(const ScheduledSlip& s) {
    return s.foot + " " + trigger_name(s.mode) + "+" + std::to_string(s.delay_ms) + " " + s.direction + " " +
           fmt(s.speed_m_s, 2) + " m/s";
}

}  // namespace

const char* slip_mode_label(SlipMode mode) {
    switch (mode) {
        case SlipMode::AfterHS: return "early (HS)";
        case SlipMode::MidStance: return "mid (FF)";
        case SlipMode::LateStance: return "late (HO)";
        default: return "none";
    }
}

SlipSchedule load_slip_schedule(const std::string& path, int trial, const Config& cfg,
                                bool require_conversion) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("cannot open schedule file: " + path);
    }

    SlipSchedule sched;
    sched.path = path;
    sched.trial = trial;
    sched.counts_per_m = cfg.slip_schedule.counts_per_m();

    std::vector<std::string> errors;
    std::map<std::string, size_t> col;
    std::set<int> seen_ids;
    std::set<double> durations;
    std::string line;
    int line_no = 0;
    bool have_header = false;
    int other_trials = 0;

    auto get = [&](const std::vector<std::string>& f, const char* name) -> std::string {
        const auto it = col.find(name);
        return (it != col.end() && it->second < f.size()) ? f[it->second] : std::string();
    };

    while (std::getline(in, line)) {
        ++line_no;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string t = trim(line);
        if (t.empty()) continue;
        if (t[0] == '#') {
            sched.comments.push_back(t);
            continue;
        }
        const auto f = split_csv(line);
        if (!have_header) {
            for (size_t i = 0; i < f.size(); ++i) col[lower(f[i])] = i;
            have_header = true;
            for (const char* req : {"slip", "trial", "foot", "type"}) {
                if (!col.count(req)) errors.push_back(std::string("missing column '") + req + "'");
            }
            if (!col.count("arm_in_trial_s")) {
                if (col.count("onset_in_trial_s")) {
                    sched.warnings.push_back(
                        "no arm_in_trial_s column: using onset_in_trial_s, which includes the planner's "
                        "simulated delivery delay unless that model was switched off. Re-export from "
                        "tools/slip_schedule_planner.html.");
                } else {
                    errors.push_back("missing column 'arm_in_trial_s'");
                }
            }
            if (!col.count("value") && !col.count("magnitude")) {
                errors.push_back("missing column 'value' (peak speed, m/s)");
            }
            if (!errors.empty()) break;
            continue;
        }

        const std::string where = "line " + std::to_string(line_no) + ": ";
        int row_trial = 0;
        if (!parse_int(get(f, "trial"), row_trial)) {
            errors.push_back(where + "bad trial '" + get(f, "trial") + "'");
            continue;
        }
        if (row_trial != trial) {
            ++other_trials;
            continue;
        }

        ScheduledSlip s;
        s.trial = row_trial;
        bool ok = true;
        auto fail = [&](const std::string& msg) {
            errors.push_back(where + msg);
            ok = false;
        };

        if (!parse_int(get(f, "slip"), s.slip) || s.slip <= 0) fail("bad slip number '" + get(f, "slip") + "'");
        else if (!seen_ids.insert(s.slip).second) fail("duplicate slip number " + get(f, "slip"));
        if (!get(f, "block").empty() && !parse_int(get(f, "block"), s.block)) fail("bad block '" + get(f, "block") + "'");

        const std::string arm = col.count("arm_in_trial_s") ? get(f, "arm_in_trial_s") : get(f, "onset_in_trial_s");
        if (!parse_double(arm, s.arm_s) || s.arm_s < 0) fail("bad arm time '" + arm + "'");

        if (!parse_foot(get(f, "foot"), s.foot)) fail("foot must be L/R/Left/Right, got '" + get(f, "foot") + "'");

        s.type = get(f, "type");
        s.mode = parse_type(s.type);
        if (s.mode == SlipMode::None) {
            fail("unknown slip type '" + s.type + "' (Early/ES/HS, Mid/MS/FF, Late/LS/HO)");
        } else if (s.mode != SlipMode::AfterHS && !cfg.slip.stance_events_available) {
            fail("type '" + s.type + "' needs FF/HO events: set gait_detection.hs_contact_detection and "
                 "gait_detection.stance_events.enabled");
        }

        const std::string unit = get(f, "unit");
        if (!unit.empty() && lower(unit).find("m/s") == std::string::npos) {
            fail("unit '" + unit + "' is not m/s: the schedule must give peak speed in m/s");
        }
        s.magnitude = get(f, "magnitude");
        const std::string value = get(f, "value");
        if (!value.empty()) {
            if (!parse_double(value, s.speed_m_s)) fail("bad value '" + value + "'");
        } else if (!trailing_number(s.magnitude, s.speed_m_s)) {
            fail("no speed: value is blank and magnitude '" + s.magnitude + "' has no number");
        }
        if (ok && (s.speed_m_s <= 0.0 || s.speed_m_s > cfg.slip_schedule.max_speed_m_s)) {
            fail("speed " + fmt(s.speed_m_s, 3) + " m/s outside (0, " +
                 fmt(cfg.slip_schedule.max_speed_m_s, 2) + "] (slip_schedule.max_speed_m_s)");
        }

        // +velocity moves the foot anterior, -velocity posterior (confirmed
        // 2026-10-02). The YAML's older words: backward = anterior, forward = posterior.
        const std::string dir = lower(get(f, "direction"));
        if (dir.empty()) {
            const std::string mid = lower(cfg.slip.mid_stance_slip_direction);
            s.direction = (s.mode == SlipMode::AfterHS) ? "anterior"
                        : (s.mode == SlipMode::LateStance) ? "posterior"
                        : (mid == "backward" || mid == "anterior") ? "anterior" : "posterior";
        } else if (dir == "anterior" || dir == "backward") {
            s.direction = "anterior";
            s.direction_from_file = true;
        } else if (dir == "posterior" || dir == "forward") {
            s.direction = "posterior";
            s.direction_from_file = true;
        } else {
            fail("direction must be anterior/posterior (or forward/backward) or blank, got '" +
                 get(f, "direction") + "'");
        }

        const std::string delay = get(f, "delay_ms");
        if (delay.empty()) {
            s.delay_ms = (s.mode == SlipMode::MidStance) ? cfg.slip.mid_stance_delay_ms
                       : (s.mode == SlipMode::LateStance) ? cfg.slip.late_stance_delay_ms
                       : cfg.slip.mode1_delay_after_hs_ms;
        } else if (parse_int(delay, s.delay_ms) && s.delay_ms >= 0 && s.delay_ms <= 2000) {
            s.delay_from_file = true;
        } else {
            fail("delay_ms must be 0..2000 or blank, got '" + delay + "'");
        }

        const std::string dur = get(f, "trial_duration_s");
        double d = 0.0;
        if (!dur.empty()) {
            if (parse_double(dur, d) && d > 0) durations.insert(d);
            else fail("bad trial_duration_s '" + dur + "'");
        }

        s.duration_ms = cfg.slip.slip_duration_ms;
        if (ok) {
            const double cps = s.speed_m_s * sched.counts_per_m;
            if (cps > std::numeric_limits<int32_t>::max()) {
                fail("speed " + fmt(s.speed_m_s, 3) + " m/s overflows counts/s");
            } else {
                const int32_t mag = static_cast<int32_t>(std::llround(cps));
                s.velocity = (s.direction == "anterior") ? mag : -mag;
                if (cfg.slip.slip_profile_acceleration > 0) {
                    s.ramp_ms = 1000.0 * mag / cfg.slip.slip_profile_acceleration;
                }
            }
        }
        if (ok) sched.slips.push_back(s);
    }

    if (!have_header) errors.push_back("no header row");
    if (errors.empty() && sched.slips.empty()) {
        errors.push_back("no slips for trial " + std::to_string(trial) +
                         (other_trials ? " (the file has other trials)" : ""));
    }
    if (sched.counts_per_m <= 0.0) {
        const std::string msg =
            "m/s -> counts/s conversion not configured: set slip_schedule.encoder_counts_per_rev, "
            "gear_ratio and wheel_diameter_m (or counts_per_m) in the YAML";
        if (require_conversion) errors.push_back(msg);
        else sched.warnings.push_back(msg + "; velocities shown as 0");
    }
    if (!errors.empty()) {
        // The same problem on every row (e.g. a cm schedule) is listed once.
        std::map<std::string, std::vector<std::string>> by_msg;
        std::vector<std::string> order;
        for (const auto& e : errors) {
            const size_t colon = e.rfind("line ", 0) == 0 ? e.find(": ") : std::string::npos;
            const std::string msg = colon == std::string::npos ? e : e.substr(colon + 2);
            if (!by_msg.count(msg)) order.push_back(msg);
            by_msg[msg].push_back(colon == std::string::npos ? "" : e.substr(0, colon));
        }
        std::string all = "schedule " + path + " (trial " + std::to_string(trial) + "):";
        for (const auto& msg : order) {
            const auto& where = by_msg[msg];
            all += "\n  ";
            if (!where.front().empty()) {
                all += where.front();
                if (where.size() > 1) all += " and " + std::to_string(where.size() - 1) + " more";
                all += ": ";
            }
            all += msg;
        }
        throw std::runtime_error(all);
    }

    std::stable_sort(sched.slips.begin(), sched.slips.end(),
                     [](const ScheduledSlip& a, const ScheduledSlip& b) { return a.arm_s < b.arm_s; });

    const double min_gap = cfg.slip_schedule.min_gap_s;
    if (!durations.empty()) {
        sched.trial_duration_s = *durations.rbegin();
        sched.trial_duration_from_file = true;
        if (durations.size() > 1) {
            sched.warnings.push_back("trial_duration_s differs between rows; using the largest, " +
                                     fmt(sched.trial_duration_s, 1) + " s");
        }
    } else {
        sched.trial_duration_s = sched.slips.back().arm_s + min_gap + cfg.slip_schedule.retry_window_s;
        sched.warnings.push_back("no trial_duration_s column: the trial ends at " +
                                 fmt(sched.trial_duration_s, 1) + " s (last arm + min gap + retry window)");
    }
    if (sched.slips.back().arm_s >= sched.trial_duration_s) {
        throw std::runtime_error("schedule " + path + ": slip " + std::to_string(sched.slips.back().slip) +
                                 " is armed after the trial ends");
    }

    int tight = 0;
    for (size_t i = 1; i < sched.slips.size(); ++i) {
        if (sched.slips[i].arm_s - sched.slips[i - 1].arm_s < min_gap - 1e-9) ++tight;
    }
    if (tight > 0) {
        sched.warnings.push_back(std::to_string(tight) + " planned arm gap(s) under min_gap_s " + fmt(min_gap, 1) +
                                 " s: those slips will be pushed back at run time");
    }

    // Peak speed reachable within the burst at the slip profile acceleration?
    std::map<double, int> short_ramp;
    for (const auto& s : sched.slips) {
        if (s.ramp_ms > s.duration_ms) ++short_ramp[s.speed_m_s];
    }
    for (const auto& [speed, n] : short_ramp) {
        const int burst_ms = cfg.slip.slip_duration_ms;
        const double reached = cfg.slip.slip_profile_acceleration * (burst_ms / 1000.0) / sched.counts_per_m;
        sched.warnings.push_back(std::to_string(n) + " slip(s) at " + fmt(speed, 2) +
                                 " m/s cannot reach peak at slip_profile_acceleration " +
                                 std::to_string(cfg.slip.slip_profile_acceleration) + ": ramp " +
                                 fmt(1000.0 * speed * sched.counts_per_m / cfg.slip.slip_profile_acceleration, 0) +
                                 " ms > burst " + std::to_string(burst_ms) + " ms (peak ~" + fmt(reached, 2) +
                                 " m/s)");
    }
    return sched;
}

void print_slip_schedule(std::ostream& os, const SlipSchedule& sched) {
    os << "Schedule " << sched.path << ", trial " << sched.trial << ": " << sched.slips.size()
       << " slips, trial length " << std::fixed << std::setprecision(1) << sched.trial_duration_s << " s ("
       << sched.trial_duration_s / 60.0 << " min)\n";
    if (sched.counts_per_m > 0) {
        os << "Conversion: " << std::setprecision(1) << sched.counts_per_m << " counts per m (1 m/s = "
           << std::setprecision(0) << sched.counts_per_m << " counts/s)\n";
    }
    for (const auto& c : sched.comments) os << "  " << c << '\n';
    os << "   slip block   arm_s  foot   type        speed    counts/s  dir        delay  ramp\n";
    for (const auto& s : sched.slips) {
        os << std::setw(7) << s.slip << std::setw(6) << s.block << std::setw(8) << std::setprecision(1) << s.arm_s
           << "  " << std::left << std::setw(6) << s.foot << std::setw(11) << slip_mode_label(s.mode) << std::right
           << std::setw(5) << std::setprecision(2) << s.speed_m_s << " m/s" << std::setw(10) << s.velocity << "  "
           << std::left << std::setw(10) << (s.direction + (s.direction_from_file ? "*" : "")) << std::right
           << std::setw(4) << s.delay_ms << (s.delay_from_file ? "*" : " ") << std::setw(5) << std::setprecision(0)
           << s.ramp_ms << " ms\n";
    }
    os << "  (* = set in the schedule file, otherwise the YAML default)\n";
    std::map<std::string, int> counts;
    for (const auto& s : sched.slips) {
        counts[condition_of(s)]++;
    }
    for (const auto& [k, n] : counts) os << "  " << std::setw(3) << n << " x " << k << '\n';
    for (const auto& w : sched.warnings) os << "WARNING: " << w << '\n';
    os.flush();
}

// ---------------------------------------------------------------------------

SlipScheduleRunner::SlipScheduleRunner(const SlipSchedule& schedule, const Params& params,
                                       SlipDeliverer* left, SlipDeliverer* right, std::ostream& events,
                                       std::ostream& console)
    : schedule_(schedule),
      params_(params),
      left_(left),
      right_(right),
      events_(events),
      console_(console),
      records_(schedule.slips.size()) {
    events_ << "trial_time_s,log_time_s,slip,attempt,event,foot,type,mode,speed_m_s,velocity_cps,direction,"
               "delay_ms,planned_arm_s,due_s,detail\n";
}

const char* SlipScheduleRunner::status_name(Status st) {
    switch (st) {
        case Status::Pending: return "pending";
        case Status::Requested: return "requested";
        case Status::Armed: return "armed";
        case Status::Triggered: return "triggered";
        case Status::Slipping: return "slipping";
        case Status::WaitRetry: return "waiting to retry";
        case Status::Delivered: return "delivered";
        case Status::Missed: return "missed";
        case Status::Aborted: return "aborted";
        case Status::Skipped: return "skipped";
        case Status::NotRun: return "not run";
    }
    return "?";
}

bool SlipScheduleRunner::is_final(Status st) {
    return st == Status::Delivered || st == Status::Missed || st == Status::Aborted || st == Status::Skipped ||
           st == Status::NotRun;
}

double SlipScheduleRunner::trial_s(int64_t ns) const {
    int64_t paused = paused_total_ns_;
    if (paused_ && ns > pause_start_ns_) paused += ns - pause_start_ns_;
    return clock_offset_s_ + static_cast<double>(ns - start_ns_ - paused) / 1e9;
}

SlipDeliverer* SlipScheduleRunner::deliverer_for(const ScheduledSlip& s) const {
    return (s.foot == "Left") ? left_ : right_;
}

std::string SlipScheduleRunner::describe(const ScheduledSlip& s) const {
    return "slip " + std::to_string(s.slip) + " (" + condition_of(s) + ")";
}

int SlipScheduleRunner::active_slip() const {
    if (current_ >= records_.size()) return 0;
    const Status st = records_[current_].status;
    return (st == Status::Pending || is_final(st)) ? 0 : schedule_.slips[current_].slip;
}

void SlipScheduleRunner::write(double t, int64_t ns, const ScheduledSlip* s, int attempt, const std::string& event,
                               const std::string& detail) {
    events_ << fmt(t, 3) << ',' << fmt(static_cast<double>(ns - log_origin_ns_) / 1e9, 3) << ',';
    if (s) {
        const Record& r = records_[static_cast<size_t>(s - schedule_.slips.data())];
        events_ << s->slip << ',' << attempt << ',' << event << ',' << s->foot << ',' << csv_field(s->type) << ','
                << slip_mode_label(s->mode) << ',' << fmt(s->speed_m_s, 3) << ',' << s->velocity << ','
                << s->direction << ',' << s->delay_ms << ',' << fmt(s->arm_s, 3) << ','
                << (r.due_s >= 0 ? fmt(r.due_s, 3) : "") << ',';
    } else {
        events_ << ",," << event << ",,,,,,,,,,";
    }
    events_ << csv_field(detail) << '\n';
    events_.flush();
}

void SlipScheduleRunner::say(double t, const std::string& text) {
    console_ << "[sched] t=" << std::fixed << std::setprecision(1) << std::setw(6) << t << " s  " << text << '\n';
    console_.flush();
}

void SlipScheduleRunner::start_from_slip(int slip) {
    if (started_) return;
    size_t first = schedule_.slips.size();
    for (size_t i = 0; i < schedule_.slips.size(); ++i) {
        if (schedule_.slips[i].slip == slip) first = i;
    }
    if (first == schedule_.slips.size()) {
        throw std::runtime_error("slip " + std::to_string(slip) + " is not in trial " +
                                 std::to_string(schedule_.trial));
    }
    for (size_t i = 0; i < first; ++i) records_[i].status = Status::Skipped;
    current_ = first;
    clock_offset_s_ = std::max(0.0, schedule_.slips[first].arm_s - params_.min_gap_s);
}

void SlipScheduleRunner::start(int64_t now_ns, int64_t log_origin_ns) {
    if (started_) return;
    started_ = true;
    start_ns_ = now_ns;
    last_ns_ = now_ns;
    log_origin_ns_ = log_origin_ns;
    last_trial_s_ = clock_offset_s_;
    write(clock_offset_s_, now_ns, nullptr, 0, "trial_start",
          "trial " + std::to_string(schedule_.trial) + ", " + std::to_string(schedule_.slips.size()) +
              " slips, ends at " + fmt(schedule_.trial_duration_s, 1) + " s");
    for (size_t i = 0; i < current_; ++i) {
        write(clock_offset_s_, now_ns, &schedule_.slips[i], 0, "skipped", "resumed from a later slip");
    }
    say(clock_offset_s_, "trial " + std::to_string(schedule_.trial) + " started" +
                             (current_ > 0 ? " at slip " + std::to_string(schedule_.slips[current_].slip) : ""));
    if (current_ < records_.size()) {
        records_[current_].due_s = schedule_.slips[current_].arm_s;
        say(clock_offset_s_, "next: " + describe(schedule_.slips[current_]) + " at " +
                                 fmt(records_[current_].due_s, 1) + " s");
    }
}

void SlipScheduleRunner::send_request(size_t i, double t) {
    const ScheduledSlip& s = schedule_.slips[i];
    Record& r = records_[i];
    ++r.attempts;
    r.status = Status::Requested;
    SlipRequest req;
    req.mode = s.mode;
    req.velocity = s.velocity;
    req.delay_ms = s.delay_ms;
    req.duration_ms = s.duration_ms;
    req.id = s.slip;
    deliverer_for(s)->request(req);
    write(t, last_ns_, &s, r.attempts, "request", r.attempts > 1 ? "retry after: " + r.last_reason : "");
    say(t, describe(s) + ": armed, attempt " + std::to_string(r.attempts));
}

void SlipScheduleRunner::handle_outcome(const SlipOutcome& o) {
    size_t i = schedule_.slips.size();
    for (size_t k = 0; k < schedule_.slips.size(); ++k) {
        if (schedule_.slips[k].slip == o.id) i = k;
    }
    if (i == schedule_.slips.size()) return;
    const ScheduledSlip& s = schedule_.slips[i];
    Record& r = records_[i];
    const double t = trial_s(o.time_ns);
    const std::string kind = SlipPerturbationNode::outcome_name(o.kind);
    if (i != current_ || is_final(r.status)) {
        // Not expected; a Cancelled here is just the echo of our own cancel()
        // after the slip was given up, already recorded.
        if (o.kind != SlipOutcome::Kind::Cancelled) {
            write(t, o.time_ns, &s, r.attempts, "late_" + kind, o.detail);
        }
        return;
    }
    using K = SlipOutcome::Kind;
    switch (o.kind) {
        case K::Armed:
            r.status = Status::Armed;
            write(t, o.time_ns, &s, r.attempts, kind, "");
            break;
        case K::Triggered:
            r.status = Status::Triggered;
            write(t, o.time_ns, &s, r.attempts, kind,
                  o.detail + " at " + fmt(trial_s(o.event_ns), 3) + " s");
            break;
        case K::Started:
            r.status = Status::Slipping;
            r.onset_s = t;
            last_onset_s_ = t;
            write(t, o.time_ns, &s, r.attempts, kind, fmt(t - s.arm_s, 3) + " s after the planned arm");
            say(t, describe(s) + ": FIRED");
            break;
        case K::Completed:
            finish_current(Status::Delivered, t, "delivered", "");
            break;
        case K::Refused:
        case K::Cancelled:
        case K::Disarmed:
            r.status = Status::WaitRetry;
            r.last_reason = kind + ": " + o.detail;
            r.next_attempt_s = t + (o.kind == K::Cancelled ? 0.0 : params_.refused_retry_s);
            write(t, o.time_ns, &s, r.attempts, kind, o.detail);
            say(t, describe(s) + ": " + kind + " (" + o.detail + "), will retry");
            break;
        case K::Aborted:
            if (o.during_slip) {
                finish_current(Status::Aborted, t, "aborted", o.detail + " during the burst; not repeated");
            } else {
                r.status = Status::WaitRetry;
                r.last_reason = "aborted: " + o.detail;
                r.next_attempt_s = t;
                write(t, o.time_ns, &s, r.attempts, kind, o.detail + " before the burst");
                say(t, describe(s) + ": aborted before the burst (" + o.detail + "), will retry");
            }
            break;
    }
}

void SlipScheduleRunner::finish_current(Status st, double t, const std::string& event, const std::string& reason) {
    const ScheduledSlip& s = schedule_.slips[current_];
    Record& r = records_[current_];
    r.status = st;
    if (!reason.empty()) r.last_reason = reason;
    write(t, last_ns_, &s, r.attempts, event, reason);
    say(t, describe(s) + ": " + status_name(st) + (reason.empty() ? "" : " (" + reason + ")"));
    ++current_;
    if (current_ < records_.size()) {
        const ScheduledSlip& n = schedule_.slips[current_];
        records_[current_].due_s = std::max(n.arm_s, last_onset_s_ + params_.min_gap_s);
        say(t, "next: " + describe(n) + " at " + fmt(records_[current_].due_s, 1) + " s" +
                   (records_[current_].due_s > n.arm_s + 1e-6
                        ? " (planned " + fmt(n.arm_s, 1) + " s, pushed back for the min gap)"
                        : ""));
    } else {
        say(t, "all slips done; trial ends at " + fmt(schedule_.trial_duration_s, 1) + " s");
    }
}

void SlipScheduleRunner::end_trial(double t, const std::string& reason) {
    for (size_t i = current_; i < records_.size(); ++i) {
        Record& r = records_[i];
        if (is_final(r.status)) continue;
        const ScheduledSlip& s = schedule_.slips[i];
        if (r.status == Status::Requested || r.status == Status::Armed || r.status == Status::Triggered) {
            deliverer_for(s)->cancel(reason);
        }
        if (r.status == Status::Slipping) {
            r.status = Status::Aborted;   // stop(): the app stops the drives right after
        } else {
            r.status = (r.status == Status::Pending) ? Status::NotRun : Status::Missed;
        }
        r.last_reason = reason;
        write(t, last_ns_, &s, r.attempts, status_name(r.status), reason);
    }
    current_ = records_.size();
    write(t, last_ns_, nullptr, 0, "trial_end", reason);
    say(t, "trial ended: " + reason);
    finished_ = true;
}

void SlipScheduleRunner::tick(int64_t now_ns, bool paused) {
    if (!started_ || finished_) return;
    last_ns_ = now_ns;
    if (paused && !paused_) {
        paused_ = true;
        pause_start_ns_ = now_ns;
        write(trial_s(now_ns), now_ns, nullptr, 0, "paused", "emergency stop: trial clock stopped");
        say(trial_s(now_ns), "PAUSED (emergency stop): trial clock stopped, 'r' resumes");
    } else if (!paused && paused_) {
        paused_total_ns_ += now_ns - pause_start_ns_;
        paused_ = false;
        write(trial_s(now_ns), now_ns, nullptr, 0, "resumed", "");
        say(trial_s(now_ns), "resumed");
    }
    const double t = trial_s(now_ns);
    last_trial_s_ = t;

    for (SlipDeliverer* d : {left_, right_}) {
        if (!d) continue;
        for (const auto& o : d->take_outcomes()) handle_outcome(o);
    }

    if (t >= schedule_.trial_duration_s) {
        if (current_ < records_.size() &&
            (records_[current_].status == Status::Triggered || records_[current_].status == Status::Slipping)) {
            return;   // let a firing slip finish
        }
        end_trial(t, "trial duration reached");
        return;
    }
    if (current_ >= records_.size()) return;

    const ScheduledSlip& s = schedule_.slips[current_];
    Record& r = records_[current_];
    const double deadline = r.due_s + params_.retry_window_s;
    switch (r.status) {
        case Status::Pending:
            if (!paused && t >= r.due_s) send_request(current_, t);
            break;
        case Status::WaitRetry:
            if (t > deadline) {
                finish_current(Status::Missed, t, "missed", "retry window over; last: " + r.last_reason);
            } else if (!paused && t >= r.next_attempt_s) {
                send_request(current_, t);
            }
            break;
        case Status::Requested:
        case Status::Armed:
            if (t > deadline) {
                deliverer_for(s)->cancel("retry window over");
                finish_current(Status::Missed, t, "missed",
                               "retry window over while waiting for the trigger event" +
                                   (r.last_reason.empty() ? "" : "; last: " + r.last_reason));
            }
            break;
        default:
            break;
    }
}

void SlipScheduleRunner::stop(int64_t now_ns, const std::string& reason) {
    if (!started_ || finished_) {
        finished_ = true;
        return;
    }
    last_ns_ = now_ns;
    end_trial(trial_s(now_ns), reason);
}

void SlipScheduleRunner::print_summary(std::ostream& os) const {
    std::map<Status, int> by_status;
    struct Combo { int planned = 0, delivered = 0, missed = 0, aborted = 0, other = 0; };
    std::map<std::string, Combo> combos;
    std::vector<double> late;
    double prev_onset = -1.0;
    double min_gap = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < records_.size(); ++i) {
        const ScheduledSlip& s = schedule_.slips[i];
        const Record& r = records_[i];
        by_status[r.status]++;
        Combo& c = combos[condition_of(s)];
        c.planned++;
        if (r.status == Status::Delivered) c.delivered++;
        else if (r.status == Status::Missed) c.missed++;
        else if (r.status == Status::Aborted) c.aborted++;
        else c.other++;
        if (r.onset_s >= 0) {
            late.push_back(r.onset_s - s.arm_s);
            if (prev_onset >= 0) min_gap = std::min(min_gap, r.onset_s - prev_onset);
            prev_onset = r.onset_s;
        }
    }
    os << "\n=== Trial " << schedule_.trial << " summary: " << records_.size() << " slips scheduled\n";
    for (const auto& [st, n] : by_status) os << "  " << std::setw(16) << status_name(st) << ": " << n << '\n';
    os << "  condition                              planned delivered missed aborted other\n";
    for (const auto& [k, c] : combos) {
        os << "  " << std::left << std::setw(38) << k << std::right << std::setw(8) << c.planned << std::setw(10)
           << c.delivered << std::setw(7) << c.missed << std::setw(8) << c.aborted << std::setw(6) << c.other << '\n';
    }
    if (!late.empty()) {
        std::sort(late.begin(), late.end());
        os << "  onset after planned arm: median " << fmt(late[late.size() / 2], 2) << " s, max "
           << fmt(late.back(), 2) << " s\n";
    }
    if (std::isfinite(min_gap)) os << "  shortest gap between fired slips: " << fmt(min_gap, 2) << " s\n";
    os.flush();
}

}  // namespace motorized_shoe
