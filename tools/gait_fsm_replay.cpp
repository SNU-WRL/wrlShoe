// Offline replay of the gait FSM against logged IMU samples.
//
//   gait_fsm_replay <params.yaml> <log.csv> [<log.csv> ...] > events.csv
//
// Every log is run twice per foot through GaitEventFSM configured exactly as
// GaitPhaseDetectionNode configures it: once with the trough-recovery HS
// detector ("old") and once with gait_detection.hs_contact_detection forced on
// ("contact"), both using the thresholds / hs_contact block of <params.yaml>. An
// IMU sample is a row whose imu_<foot>_msg_count changed, the same rule the
// gait node uses, stamped with the row's time_s. Output, one row per event:
//   log,foot,mode,label,t_detect_s,t_event_s
// t_detect_s = the sample on which the FSM declared the event (when a slip
// would fire), t_event_s = the (possibly back-dated) event timestamp.
// scripts/eval_hs_replay.py scores the result against contact markers.
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/gait_fsm.hpp"

using motorized_shoe::Config;
using motorized_shoe::GaitEventFSM;

namespace {

struct Sample {
    double t = 0.0;
    float ax = 0, ay = 0, az = 0, gz = 0;
};

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) out.push_back(cell);
    return out;
}

int column(const std::vector<std::string>& header, const std::string& name) {
    for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) return static_cast<int>(i);
    }
    std::cerr << "missing column " << name << '\n';
    std::exit(2);
}

void configure(GaitEventFSM& fsm, const Config& cfg, bool contact) {
    fsm.set_thresholds(cfg.gait_thresholds.hs_threshold, cfg.gait_thresholds.to_threshold,
                       cfg.gait_thresholds.min_swing_dwell_ms);
    fsm.set_state_timeout_ms(cfg.gait_state_timeout_ms);
    fsm.set_hs_accel_veto(cfg.gait_thresholds.hs_accel_veto,
                          cfg.gait_thresholds.hs_impact_threshold);
    fsm.set_filter_window(cfg.gait_ma_window);
    fsm.set_contact_hs(contact, cfg.gait_hs_contact);
}

void replay(const std::string& log, const char* foot, const std::vector<Sample>& samples,
            const Config& cfg, bool contact) {
    GaitEventFSM fsm(cfg.gait_sampling_frequency, foot);
    configure(fsm, cfg, contact);
    for (const Sample& s : samples) {
        const int64_t ts = static_cast<int64_t>(s.t * 1e9);
        const auto ev = fsm.check_state_transition(s.gz, s.ax, s.ay, s.az, 0.0f, ts);
        if (ev.event_detected) {
            std::cout << log << ',' << foot << ',' << (contact ? "contact" : "old") << ','
                      << ev.event_label << ',' << s.t << ','
                      << static_cast<double>(ev.event_timestamp_ns) / 1e9 << '\n';
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " <params.yaml> <log.csv> [<log.csv> ...]\n";
        return 1;
    }
    const Config cfg = motorized_shoe::load_config(argv[1]);
    std::cout.precision(9);
    std::cout << "log,foot,mode,label,t_detect_s,t_event_s\n";

    for (int a = 2; a < argc; ++a) {
        std::ifstream in(argv[a]);
        if (!in) {
            std::cerr << "cannot open " << argv[a] << '\n';
            return 2;
        }
        std::string line;
        std::getline(in, line);
        const auto header = split(line);
        const size_t ncols = header.size();
        const int c_t = column(header, "time_s");
        const char* feet[2] = {"Left", "Right"};
        int c_cnt[2], c_ax[2], c_ay[2], c_az[2], c_gz[2];
        for (int f = 0; f < 2; ++f) {
            const std::string p = std::string("imu_") + (f == 0 ? "left_" : "right_");
            c_cnt[f] = column(header, p + "msg_count");
            c_ax[f] = column(header, p + "ax");
            c_ay[f] = column(header, p + "ay");
            c_az[f] = column(header, p + "az");
            c_gz[f] = column(header, p + "gz");
        }

        std::vector<Sample> samples[2];
        std::string last_cnt[2] = {"0", "0"};
        while (std::getline(in, line)) {
            const auto row = split(line);
            if (row.size() < ncols) continue;  // truncated last line
            for (int f = 0; f < 2; ++f) {
                if (row[c_cnt[f]] == last_cnt[f]) continue;
                last_cnt[f] = row[c_cnt[f]];
                Sample s;
                s.t = std::atof(row[c_t].c_str());
                s.ax = static_cast<float>(std::atof(row[c_ax[f]].c_str()));
                s.ay = static_cast<float>(std::atof(row[c_ay[f]].c_str()));
                s.az = static_cast<float>(std::atof(row[c_az[f]].c_str()));
                s.gz = static_cast<float>(std::atof(row[c_gz[f]].c_str()));
                samples[f].push_back(s);
            }
        }

        std::string name = argv[a];
        const size_t slash = name.find_last_of('/');
        if (slash != std::string::npos) name = name.substr(slash + 1);
        for (int f = 0; f < 2; ++f) {
            replay(name, feet[f], samples[f], cfg, false);
            replay(name, feet[f], samples[f], cfg, true);
        }
    }
    return 0;
}
