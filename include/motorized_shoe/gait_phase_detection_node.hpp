#ifndef MOTORIZED_SHOE_GAIT_PHASE_DETECTION_NODE_HPP
#define MOTORIZED_SHOE_GAIT_PHASE_DETECTION_NODE_HPP

#include <memory>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"
#include "motorized_shoe/gait_fsm.hpp"

namespace motorized_shoe {

class GaitPhaseDetectionNode {
public:
    GaitPhaseDetectionNode(const Config& cfg, DataBus& bus);
    void tick();

private:
    void process(const IMUData& imu, GaitEventFSM& fsm, const char* foot);

    DataBus& bus_;
    std::unique_ptr<GaitEventFSM> left_fsm_;
    std::unique_ptr<GaitEventFSM> right_fsm_;
    uint32_t last_left_msg_count_ = 0;
    uint32_t last_right_msg_count_ = 0;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_GAIT_PHASE_DETECTION_NODE_HPP
