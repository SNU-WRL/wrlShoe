#ifndef MOTORIZED_SHOE_GAIT_PHASE_DETECTION_NODE_HPP
#define MOTORIZED_SHOE_GAIT_PHASE_DETECTION_NODE_HPP

#include <memory>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"
#include "motorized_shoe/gait_fsm.hpp"

namespace motorized_shoe {

// Estimates the per-foot gravity vector (in the IMU's global/world frame) from
// the first N still samples and subtracts it to yield free acceleration. Until
// calibrated it falls back to subtracting 9.81 m/s^2 along global Z. If the IMU
// already streams linear/free acceleration, calibration converges to ~0 and the
// subtraction is a no-op.
struct GravityEstimator {
    int samples_needed = 60;
    int count = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    bool calibrated = false;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 9.81f;

    void update(float ax, float ay, float az);
};

class GaitPhaseDetectionNode {
public:
    GaitPhaseDetectionNode(const Config& cfg, DataBus& bus);
    void tick();

private:
    void process(const IMUData& imu, GaitEventFSM& fsm, GravityEstimator& gravity,
                 const char* foot);

    DataBus& bus_;
    std::unique_ptr<GaitEventFSM> left_fsm_;
    std::unique_ptr<GaitEventFSM> right_fsm_;
    GravityEstimator left_gravity_;
    GravityEstimator right_gravity_;
    uint32_t last_left_msg_count_ = 0;
    uint32_t last_right_msg_count_ = 0;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_GAIT_PHASE_DETECTION_NODE_HPP
