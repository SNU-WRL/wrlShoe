#ifndef MOTORIZED_SHOE_DATA_BUS_HPP
#define MOTORIZED_SHOE_DATA_BUS_HPP

#include <deque>
#include <mutex>
#include <string>

#include "motorized_shoe/types.hpp"

namespace motorized_shoe {

class DataBus {
public:
    void update_imu(const IMUData& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (data.foot == "Left") {
            state_.imu_left = data;
        } else {
            state_.imu_right = data;
        }
    }

    void update_gait(const GaitPhase& gait) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (gait.foot == "Left") {
            state_.gait_left = gait;
        } else {
            state_.gait_right = gait;
        }
    }

    // Push a gait *transition* event (called by gait_phase_detection_node only
    // when the FSM actually changes state). The buffer holds the last
    // kMaxGaitEvents per foot so a slow poller (e.g. slip_perturbation_node)
    // can recover transitions it would otherwise miss by reading only the
    // latest snapshot. The event carries the true detection timestamp.
    void push_gait_event(const GaitPhase& event) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& buf = (event.foot == "Left") ? left_events_ : right_events_;
        buf.push_back(event);
        if (buf.size() > kMaxGaitEvents) {
            buf.pop_front();
        }
    }

    // Iterate every buffered transition for `foot` whose detection_count is
    // strictly greater than `since`. Callback is invoked under the lock — keep
    // it short and non-blocking. Using a template avoids std::function alloc.
    template <typename Callback>
    void for_each_gait_event_since(
        const std::string& foot, uint32_t since, Callback&& cb) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& buf = (foot == "Left") ? left_events_ : right_events_;
        for (const auto& e : buf) {
            if (e.detection_count > since) {
                cb(e);
            }
        }
    }

    void update_command(const ElmoCommand& cmd) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (cmd.foot == "Left") {
            state_.cmd_left = cmd;
        } else {
            state_.cmd_right = cmd;
        }
    }

    void update_status(const ElmoStatus& status) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status.foot == "Left") {
            state_.status_left = status;
        } else {
            state_.status_right = status;
        }
    }

    void update_motor_info(const ElmoMotorInfo& motor) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (motor.foot == "Left") {
            state_.motor_left = motor;
        } else {
            state_.motor_right = motor;
        }
    }

    SystemSnapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        SystemSnapshot copy = state_;
        copy.timestamp_ns = now_ns();
        return copy;
    }

private:
    static constexpr size_t kMaxGaitEvents = 32;

    mutable std::mutex mutex_;
    SystemSnapshot state_;
    std::deque<GaitPhase> left_events_;
    std::deque<GaitPhase> right_events_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_DATA_BUS_HPP
