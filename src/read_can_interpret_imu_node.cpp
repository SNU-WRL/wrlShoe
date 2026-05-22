#include "motorized_shoe/read_can_interpret_imu_node.hpp"

namespace motorized_shoe {

ReadCanInterpretImuNode::ReadCanInterpretImuNode(const Config& cfg, DataBus& bus)
    : bus_(bus),
      left_ids_(cfg.imu_left_can_ids),
      right_ids_(cfg.imu_right_can_ids),
      can_socket_(std::make_unique<CANSocket>(cfg.can_imu_interface)) {}

void ReadCanInterpretImuNode::tick() {
    uint32_t can_id = 0;
    uint8_t data[8] = {0};
    size_t len = 0;

    while (can_socket_->recv_message(can_id, data, len, 0)) {
        parse_can_message(can_id, data, len);
    }
}

void ReadCanInterpretImuNode::parse_can_message(uint32_t can_id, const uint8_t* data, size_t len) {
    if (can_id == static_cast<uint32_t>(left_ids_.rotation_vector) && len >= 8) {
        left_.rv_r = get_int16(data + 0) / 10000.0f;
        left_.rv_i = get_int16(data + 2) / 10000.0f;
        left_.rv_j = get_int16(data + 4) / 10000.0f;
        left_.rv_k = get_int16(data + 6) / 10000.0f;
        ++left_.msg_count;
    } else if (can_id == static_cast<uint32_t>(left_ids_.accelerometer) && len >= 6) {
        left_.ax = get_int16(data + 0) / 1000.0f;
        left_.ay = get_int16(data + 2) / 1000.0f;
        left_.az = get_int16(data + 4) / 1000.0f;
    } else if (can_id == static_cast<uint32_t>(left_ids_.gyroscope) && len >= 6) {
        left_.gx = get_int16(data + 0) / 1000.0f;
        left_.gy = get_int16(data + 2) / 1000.0f;
        left_.gz = get_int16(data + 4) / 1000.0f;
        publish_imu("Left", left_);
    } else if (can_id == static_cast<uint32_t>(left_ids_.magnetometer) && len >= 6) {
        left_.mx = get_int16(data + 0) / 1000.0f;
        left_.my = get_int16(data + 2) / 1000.0f;
        left_.mz = get_int16(data + 4) / 1000.0f;
    }

    if (can_id == static_cast<uint32_t>(right_ids_.rotation_vector) && len >= 8) {
        right_.rv_r = get_int16(data + 0) / 10000.0f;
        right_.rv_i = get_int16(data + 2) / 10000.0f;
        right_.rv_j = get_int16(data + 4) / 10000.0f;
        right_.rv_k = get_int16(data + 6) / 10000.0f;
        ++right_.msg_count;
    } else if (can_id == static_cast<uint32_t>(right_ids_.accelerometer) && len >= 6) {
        right_.ax = get_int16(data + 0) / 1000.0f;
        right_.ay = get_int16(data + 2) / 1000.0f;
        right_.az = get_int16(data + 4) / 1000.0f;
    } else if (can_id == static_cast<uint32_t>(right_ids_.gyroscope) && len >= 6) {
        right_.gx = get_int16(data + 0) / 1000.0f;
        right_.gy = get_int16(data + 2) / 1000.0f;
        right_.gz = get_int16(data + 4) / 1000.0f;
        publish_imu("Right", right_);
    } else if (can_id == static_cast<uint32_t>(right_ids_.magnetometer) && len >= 6) {
        right_.mx = get_int16(data + 0) / 1000.0f;
        right_.my = get_int16(data + 2) / 1000.0f;
        right_.mz = get_int16(data + 4) / 1000.0f;
    }
}

void ReadCanInterpretImuNode::publish_imu(const std::string& foot, const IMUBuffer& src) {
    IMUData msg;
    msg.timestamp_ns = now_ns();
    msg.foot = foot;
    msg.rv_r = src.rv_r;
    msg.rv_i = src.rv_i;
    msg.rv_j = src.rv_j;
    msg.rv_k = src.rv_k;
    msg.ax = src.ax;
    msg.ay = src.ay;
    msg.az = src.az;
    msg.gx = src.gx;
    msg.gy = src.gy;
    msg.gz = src.gz;
    msg.mx = src.mx;
    msg.my = src.my;
    msg.mz = src.mz;
    msg.msg_count = src.msg_count;
    msg.valid = true;
    bus_.update_imu(msg);
}

}  // namespace motorized_shoe
