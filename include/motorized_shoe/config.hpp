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
    float hs_threshold = -1.3963f;  // Heel Strike: gyro_z negative-peak threshold (was -80 deg/s)
    float ts_threshold = -0.5236f;  // Toe Strike: gyro_z up-cross threshold (was -30 deg/s)
    float ho_threshold = -0.15f;    // Heel Off: gyro_z down-cross threshold
    float to_threshold = -3.4907f;  // Toe Off: gyro_z negative-peak threshold (was -200 deg/s)
    // Into-swing gyro threshold (rad/s). Original used +50 deg/s = 0.873 rad/s on
    // the forward (positive) gyro_z swing, NOT an accel magnitude.
    float swing_gyro_threshold = 0.8727f;
    // Midstance "quiet" threshold on the GRAVITY-FREE accel norm (m/s^2). The old
    // value of 12 was inflated only because the raw accel still carried gravity;
    // on free acceleration the foot is near-zero at rest, so 3 is correct.
    float midstance_threshold = 3.0f;
    // Minimum dwell in Swing before a heel strike may fire, so the HS peak
    // detector can't latch onto an early-swing gyro dip.
    int min_swing_dwell_ms = 150;
};

struct Config {
    std::string can_imu_interface = "can1";
    std::string can_elmo_interface = "can0";

    IMUCanIds imu_left_can_ids{0x110, 0x111, 0x112, 0x113};
    IMUCanIds imu_right_can_ids{0x120, 0x121, 0x122, 0x123};

    int elmo_node_left = 127;
    int elmo_node_right = 126;

    int loop_frequency_hz = 1000;

    // ELMO profile acceleration / deceleration (counts/sec^2) written to 0x6083 /
    // 0x6084 at drive init. Defaults match the historic hardcoded 1e6 so behavior
    // is unchanged when elmo_config.profile_{accel,decel} are absent from the YAML.
    int32_t profile_acceleration = 1000000;
    int32_t profile_deceleration = 1000000;

    float gait_sampling_frequency = 120.0f;
    bool gait_use_both_feet = false;
    GaitThresholds gait_thresholds;

    // Length (samples) of the moving-average filter applied to gyro_z and the
    // free-accel norm feeding the FSM. The original used int(fs*0.05) = 6 at
    // 120 Hz. The filter's group delay of (window-1)/2 samples is compensated
    // for in the back-dated event timestamps.
    int gait_ma_window = 6;
    // Number of still samples (|global accel| in [9,11] m/s^2) averaged at
    // startup to estimate the per-foot gravity vector that is subtracted to
    // produce free acceleration. ~0.5 s at 120 Hz.
    int gravity_calib_samples = 60;

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
