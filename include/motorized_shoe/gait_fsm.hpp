#ifndef MOTORIZED_SHOE_GAIT_FSM_HPP
#define MOTORIZED_SHOE_GAIT_FSM_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace motorized_shoe {

enum class GaitState {
    MidStance,
    HeelOff,
    ToeOff,
    Swing,
    HeelStrike,
    ToeStrike
};

const char* gait_state_to_string(GaitState state);

struct GaitFSMState {
    GaitState current_state = GaitState::MidStance;
    int searching_start_idx = 0;
    uint32_t detection_count = 0;
};

class GaitEventFSM {
public:
    GaitEventFSM(float fs = 120.0f, const std::string& foot = "Right");

    void set_thresholds(float hs, float ts, float ho, float to, float swing, float midstance,
                        float impact, int min_swing_dwell_ms);

    struct GaitEvent {
        GaitState state = GaitState::MidStance;
        float gyro_z_value = 0.0f;
        uint32_t detection_count = 0;
        bool event_detected = false;
    };

    GaitEvent check_state_transition(float gyro_z, float accel_norm, float foot_angle);

private:
    float fs_;
    std::string foot_;
    GaitFSMState fsm_state_;

    float hs_threshold_ = -80.0f;
    float ts_threshold_ = -30.0f;
    float ho_threshold_ = -30.0f;
    float to_threshold_ = -200.0f;
    float swing_threshold_ = 50.0f;
    float midstance_threshold_ = 3.0f;
    float impact_threshold_ = 20.0f;
    int min_swing_samples_ = 18;  // ~150 ms at 120 Hz

    // Per-swing gating state. Resets on TO->Swing entry.
    bool saw_impact_peak_ = false;
    int swing_samples_ = 0;

    std::deque<float> gyro_buffer_;
    std::deque<float> accel_buffer_;

    static constexpr size_t BUFFER_SIZE = 200;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_GAIT_FSM_HPP
