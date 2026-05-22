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
            << "imu_node_latency_us,status_node_latency_us,gait_node_latency_us,command_node_latency_us,loop_latency_us"
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
          << s.loop_latency_us
          << '\n';
}

}  // namespace motorized_shoe
