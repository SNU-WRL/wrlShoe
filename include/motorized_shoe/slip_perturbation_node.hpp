#ifndef MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP
#define MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include "motorized_shoe/config.hpp"
#include "motorized_shoe/data_bus.hpp"
#include "motorized_shoe/send_can_command_to_elmo_node.hpp"

namespace motorized_shoe {

// Slip perturbation controller. Drives the ELMO command node to deliver
// one-shot slip velocity bursts timed relative to gait events.
//
//   Mode AfterHS  -> fires `mode1_delay_after_hs_ms` after the next HS event.
//   Mode BeforeTO -> fires `mode2_delay_after_ho_ms` after the next HO event
//                    (HO precedes TO physically by ~50-150 ms, used as the
//                    "slightly before toe-off" trigger).
//
// Single-threaded: tick() runs in the main control loop. request_slip() is
// safe to call from another thread (keyboard handler).
class SlipPerturbationNode {
public:
    enum class Mode {
        None = 0,
        AfterHS = 1,
        BeforeTO = 2,
    };

    SlipPerturbationNode(const SlipConfig& cfg, DataBus& bus, SendCanCommandToElmoNode& cmd_node);

    void tick();
    void request_slip(Mode mode);

    bool is_active() const;

private:
    enum class State {
        Idle,
        ArmedAfterHS,
        ArmedBeforeTO,
        DelayingBeforeSlip,
        Slipping,
    };

    void start_slip_now();
    void end_slip_now();
    void scan_for_trigger_event(const char* trigger_phase, int delay_ms);

    const SlipConfig cfg_;
    DataBus& bus_;
    SendCanCommandToElmoNode& cmd_node_;

    std::atomic<int> pending_request_{0};  // Mode value
    State state_ = State::Idle;
    Mode current_mode_ = Mode::None;
    std::chrono::steady_clock::time_point timer_deadline_{};
    uint32_t last_seen_detection_count_ = 0;
    bool detection_count_initialized_ = false;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_SLIP_PERTURBATION_NODE_HPP
