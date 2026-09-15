#ifndef MOTORIZED_SHOE_IMU_WATCHDOG_HPP
#define MOTORIZED_SHOE_IMU_WATCHDOG_HPP

#include <cstdint>
#include <iostream>

#include "motorized_shoe/types.hpp"

namespace motorized_shoe {

// Per-foot IMU freshness monitor. Both IMUs have silently died mid-run
// (right foot 2026-09-08 at 46.7 s, left foot 2026-09-11) with nothing in the
// console; the gait FSM simply froze and the slip never fired. This prints a
// line on every stale/fresh transition so the operator sees it immediately,
// and exposes the age so the slip node can refuse to arm on a dead sensor.
class ImuWatchdog {
public:
    explicit ImuWatchdog(int stale_ms) : stale_ns_(static_cast<int64_t>(stale_ms) * 1000000LL) {}

    // Age of the newest sample in ns, or -1 if the foot has never reported.
    static int64_t age_ns(const IMUData& imu, int64_t now) {
        return imu.valid ? (now - imu.timestamp_ns) : -1;
    }

    bool is_stale(const IMUData& imu, int64_t now) const {
        const int64_t age = age_ns(imu, now);
        return age < 0 || age > stale_ns_;
    }

    // Call once per tick with the current snapshot; logs transitions.
    void check(const SystemSnapshot& s) {
        const int64_t now = s.timestamp_ns;
        if (first_check_ns_ == 0) {
            first_check_ns_ = now;
        }
        check_foot("Left", s.imu_left, now, left_stale_, left_reported_);
        check_foot("Right", s.imu_right, now, right_stale_, right_reported_);
        check_node_status("Left", s.imu_left.node_status, left_last_status_);
        check_node_status("Right", s.imu_right.node_status, right_last_status_);
        // A foot that never sends a single frame would otherwise stay silent
        // forever (the left IMU on 2026-09-11 was dead from the start).
        if (!startup_reported_ && now - first_check_ns_ > kStartupGraceNs) {
            startup_reported_ = true;
            if (!left_reported_) {
                std::cerr << "[imu] Left IMU: NO frames received in the first "
                          << (kStartupGraceNs / 1000000000LL) << " s\n";
            }
            if (!right_reported_) {
                std::cerr << "[imu] Right IMU: NO frames received in the first "
                          << (kStartupGraceNs / 1000000000LL) << " s\n";
            }
        }
    }

private:
    void check_foot(const char* foot, const IMUData& imu, int64_t now, bool& stale_flag,
                    bool& ever_reported) {
        const int64_t age = age_ns(imu, now);
        if (age >= 0) {
            ever_reported = true;
        }
        if (!ever_reported) {
            // Startup grace: nothing to say until the first frame lands (or
            // the grace period expires, handled by the caller printing once).
            return;
        }
        const bool stale = age > stale_ns_;
        if (stale && !stale_flag) {
            std::cerr << "[imu] " << foot << " IMU STALE: no frame for " << (age / 1000000)
                      << " ms (msg_count " << imu.msg_count << ")\n";
        } else if (!stale && stale_flag) {
            std::cerr << "[imu] " << foot << " IMU back (msg_count " << imu.msg_count << ")\n";
        }
        stale_flag = stale;
    }

    // Teensy status frame: report every change of the reset / timeout
    // counters and a gyro rate that is not ~100 Hz.
    void check_node_status(const char* foot, const ImuNodeStatus& st, ImuNodeStatus& last) {
        if (!st.valid || st.timestamp_ns == last.timestamp_ns) {
            return;
        }
        if (!last.valid) {
            std::cerr << "[imu] " << foot << " node status online: " << st.gyro_hz
                      << " gyro frames/s, resets " << st.resets << ", timeouts " << st.timeouts
                      << ", tx dropped " << st.tx_dropped << "\n";
        } else {
            if (st.resets != last.resets) {
                std::cerr << "[imu] " << foot << " SENSOR RESET seen by the Teensy (count "
                          << st.resets << "); reports re-enabled\n";
            }
            if (st.timeouts != last.timeouts) {
                std::cerr << "[imu] " << foot << " sensor TIMEOUT recovery by the Teensy (count "
                          << st.timeouts << ")\n";
            }
            if (st.tx_dropped != last.tx_dropped) {
                std::cerr << "[imu] Teensy CAN tx drops now " << st.tx_dropped << "\n";
            }
            if ((st.gyro_hz < 90 || st.gyro_hz > 110) && (last.gyro_hz >= 90 && last.gyro_hz <= 110)) {
                std::cerr << "[imu] " << foot << " gyro rate " << st.gyro_hz << " frames/s\n";
            }
        }
        last = st;
    }

    static constexpr int64_t kStartupGraceNs = 3000000000LL;  // 3 s

    int64_t stale_ns_;
    int64_t first_check_ns_ = 0;
    bool startup_reported_ = false;
    bool left_stale_ = false;
    bool right_stale_ = false;
    bool left_reported_ = false;
    bool right_reported_ = false;
    ImuNodeStatus left_last_status_;
    ImuNodeStatus right_last_status_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_IMU_WATCHDOG_HPP
