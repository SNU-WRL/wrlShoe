#ifndef MOTORIZED_SHOE_READ_CAN_INTERPRET_IMU_NODE_HPP
#define MOTORIZED_SHOE_READ_CAN_INTERPRET_IMU_NODE_HPP

#include <memory>
#include <string>

#include "motorized_shoe/can_utils.hpp"
#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"

namespace motorized_shoe {

class ReadCanInterpretImuNode {
public:
    ReadCanInterpretImuNode(const Config& cfg, DataBus& bus);
    void tick();

private:
    struct IMUBuffer {
        float rv_r = 0.0f;
        float rv_i = 0.0f;
        float rv_j = 0.0f;
        float rv_k = 0.0f;
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;
        float mx = 0.0f;
        float my = 0.0f;
        float mz = 0.0f;
        uint32_t msg_count = 0;
    };

    void parse_can_message(uint32_t can_id, const uint8_t* data, size_t len);
    void publish_imu(const std::string& foot, const IMUBuffer& src);

    DataBus& bus_;
    IMUCanIds left_ids_;
    IMUCanIds right_ids_;
    IMUBuffer left_;
    IMUBuffer right_;
    std::unique_ptr<CANSocket> can_socket_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_READ_CAN_INTERPRET_IMU_NODE_HPP
