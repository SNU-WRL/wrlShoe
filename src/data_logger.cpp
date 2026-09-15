#include "motorized_shoe/data_logger.hpp"

#include <iomanip>
#include <stdexcept>

namespace motorized_shoe {

DataLogger::DataLogger(const std::string& path) : file_(path, std::ios::out | std::ios::trunc) {
    if (!file_.is_open()) {
        throw std::runtime_error("Failed to open log file: " + path);
    }

    file_ << "time_s,"
          << "imu_left_msg_count,imu_left_ax,imu_left_ay,imu_left_az,imu_left_gx,imu_left_gy,imu_left_gz,"
          << "imu_right_msg_count,imu_right_ax,imu_right_ay,imu_right_az,imu_right_gx,imu_right_gy,imu_right_gz,"
          << "gait_left_phase,gait_left_detection_count,gait_left_gyro_z,"
          << "gait_right_phase,gait_right_detection_count,gait_right_gyro_z,"
          << "cmd_left_velocity,cmd_left_type,cmd_right_velocity,cmd_right_type,"
            << "status_left_word,status_left_fault,status_right_word,status_right_fault,"
            << "imu_node_latency_us,status_node_latency_us,gait_node_latency_us,command_node_latency_us,loop_latency_us,log_latency_us,"
            << "motor_left_position,motor_left_velocity,motor_left_current,"
            << "motor_left_velocity_demand,motor_left_current_demand,"
            << "motor_right_position,motor_right_velocity,motor_right_current,"
            << "motor_right_velocity_demand,motor_right_current_demand,"
            // Appended 2026-09-12: IMU sample age at snapshot time (ms, -1 =
            // never received) and the drives' CiA-402 error code (0x603F /
            // EMCY) of the current or last fault (0 = none).
            << "imu_left_age_ms,imu_right_age_ms,status_left_error,status_right_error,"
            // Appended 2026-09-13: Teensy IMU-node health (1 Hz status frame):
            // gyro frames/s, sensor resets, timeout recoveries, CAN tx drops.
            // All 0 with firmware that does not send the frame.
            << "imu_left_node_hz,imu_left_node_resets,imu_left_node_timeouts,imu_left_node_tx_dropped,"
            << "imu_right_node_hz,imu_right_node_resets,imu_right_node_timeouts,imu_right_node_tx_dropped"
          << '\n';

    file_ << std::fixed << std::setprecision(6);
}

DataLogger::~DataLogger() {
    flush();
}

void DataLogger::queue_snapshot(const SystemSnapshot& s) {
    pending_.push_back(s);
}

void DataLogger::flush() {
    for (const auto& snapshot : pending_) {
        write_row(snapshot);
    }
    pending_.clear();

    if (file_.is_open()) {
        file_.flush();
    }
}

void DataLogger::write_row(const SystemSnapshot& s) {
    if (start_time_ns_ < 0) {
        start_time_ns_ = s.timestamp_ns;
    }

    const double time_s = static_cast<double>(s.timestamp_ns - start_time_ns_) / 1e9;
    auto age_ms = [&](const IMUData& imu) {
        return imu.valid ? static_cast<double>(s.timestamp_ns - imu.timestamp_ns) / 1e6 : -1.0;
    };

    file_ << time_s << ','
          << s.imu_left.msg_count << ',' << s.imu_left.ax << ',' << s.imu_left.ay << ',' << s.imu_left.az << ','
          << s.imu_left.gx << ',' << s.imu_left.gy << ',' << s.imu_left.gz << ','
          << s.imu_right.msg_count << ',' << s.imu_right.ax << ',' << s.imu_right.ay << ',' << s.imu_right.az << ','
          << s.imu_right.gx << ',' << s.imu_right.gy << ',' << s.imu_right.gz << ','
          << s.gait_left.phase << ',' << s.gait_left.detection_count << ',' << s.gait_left.gyro_z_value << ','
          << s.gait_right.phase << ',' << s.gait_right.detection_count << ',' << s.gait_right.gyro_z_value << ','
          << s.cmd_left.target_velocity << ',' << static_cast<int>(s.cmd_left.command_type) << ','
          << s.cmd_right.target_velocity << ',' << static_cast<int>(s.cmd_right.command_type) << ','
          << s.status_left.status_word << ',' << (s.status_left.fault ? 1 : 0) << ','
          << s.status_right.status_word << ',' << (s.status_right.fault ? 1 : 0) << ','
          << s.imu_node_latency_us << ',' << s.status_node_latency_us << ','
          << s.gait_node_latency_us << ',' << s.command_node_latency_us << ','
          << s.loop_latency_us << ',' << s.log_latency_us << ','
          << s.motor_left.position << ',' << s.motor_left.velocity << ',' << s.motor_left.current << ','
          << s.motor_left.velocity_demand << ',' << s.motor_left.current_demand << ','
          << s.motor_right.position << ',' << s.motor_right.velocity << ',' << s.motor_right.current << ','
          << s.motor_right.velocity_demand << ',' << s.motor_right.current_demand << ','
          << age_ms(s.imu_left) << ',' << age_ms(s.imu_right) << ','
          << s.status_left.error_code << ',' << s.status_right.error_code << ','
          << s.imu_left.node_status.gyro_hz << ',' << s.imu_left.node_status.resets << ','
          << s.imu_left.node_status.timeouts << ',' << s.imu_left.node_status.tx_dropped << ','
          << s.imu_right.node_status.gyro_hz << ',' << s.imu_right.node_status.resets << ','
          << s.imu_right.node_status.timeouts << ',' << s.imu_right.node_status.tx_dropped
          << '\n';
}

}  // namespace motorized_shoe
