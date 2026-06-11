#ifndef MOTORIZED_SHOE_CONFIG_HPP
#define MOTORIZED_SHOE_CONFIG_HPP

#include <cstdint>
#include <string>
#include <unordered_map>

namespace motorized_shoe {

struct IMUCanIds {
    int rotation_vector = 0;
    int accelerometer = 0;
    int gyroscope = 0;
    int magnetometer = 0;
};

struct SlipConfig {
    bool enabled = false;
    std::string foot = "Right";          // "Left" or "Right"
    int32_t slip_velocity = 100000;       // counts/sec sent to ELMO during slip
    int slip_duration_ms = 150;           // how long the slip lasts
    int mode1_delay_after_hs_ms = 100;    // mode 1: slip starts this long after HS
    int mode2_delay_after_mst_ms = 200;   // mode 2: slip starts this long after MSt entry
                                          // (MSt -> TO is ~200-450 ms; tune to land just before TO)
    // Profile acceleration / deceleration (counts/sec^2) written to the drive
    // when slip mode is active. The init sequence sets 1e6 by default, which
    // gives a ~150 ms ramp for a 150k slip — most of a short slip burst would
    // be spent ramping. 1e7 collapses the ramp to ~15 ms.
    int32_t slip_profile_acceleration = 10000000;
    char mode1_key = '1';
    char mode2_key = '2';
};

struct GaitThresholds {
    // Gyro thresholds (rad/s): converted from deg/s
    // -80 deg/s = -1.3963 rad/s, -30 deg/s = -0.5236 rad/s, -200 deg/s = -3.4907 rad/s
    float hs_threshold = -1.3963f;  // Heel Strike threshold (was -80 deg/s)
    float ts_threshold = -0.5236f;  // Toe Strike threshold (was -30 deg/s)
    float ho_threshold = -0.5236f;  // Heel Off threshold (was -30 deg/s)
    float to_threshold = -3.4907f;  // Toe Off threshold (was -200 deg/s)
    // Accel thresholds (m/s^2): tuned to actual walking data
    // observed accel norm: median ~9.7, max ~46 (includes gravity)
    float swing_threshold = 25.0f;  // Swing detection (reduced from 50.0 to match data)
    float midstance_threshold = 12.0f;  // Midstance window (increased from 3.0)
    // Heel-strike impact gating. Without these the Swing->HS transition fires as
    // soon as accel_norm dips below midstance_threshold during mid-swing (the
    // foot's accel norm returns to ~gravity well before the foot actually lands),
    // causing slip-perturbation commands to fire during swing.
    float impact_threshold = 20.0f;     // min accel_norm peak required during swing before HS allowed
    int min_swing_dwell_ms = 150;       // min time in Swing before HS allowed
};

struct Config {
    std::string can_imu_interface = "can1";
    std::string can_elmo_interface = "can0";

    IMUCanIds imu_left_can_ids{0x110, 0x111, 0x112, 0x113};
    IMUCanIds imu_right_can_ids{0x120, 0x121, 0x122, 0x123};

    int elmo_node_left = 127;
    int elmo_node_right = 126;

    int loop_frequency_hz = 1000;

    float gait_sampling_frequency = 120.0f;
    bool gait_use_both_feet = false;
    GaitThresholds gait_thresholds;

    std::unordered_map<std::string, int32_t> velocity_map{
        {"MSt", 0}, {"HO", 0}, {"TSt", 0}, {"TO", 50000}, {"Swing", 200000}, {"HS", 0}};

    SlipConfig slip;

    bool log_imu = false;
    bool log_gait = true;
    bool log_command = true;
    bool log_status = true;
};

Config load_config(const std::string& path);

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_CONFIG_HPP
