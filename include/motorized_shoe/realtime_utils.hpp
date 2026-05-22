#ifndef MOTORIZED_SHOE_REALTIME_UTILS_HPP
#define MOTORIZED_SHOE_REALTIME_UTILS_HPP

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace motorized_shoe {

// Promotes the calling thread to SCHED_FIFO at the given priority (1..99).
// Requires CAP_SYS_NICE or root; on failure logs a warning and leaves the
// thread on the default scheduler.
inline void set_realtime_priority(int priority = 95, const char* label = "main") {
    struct sched_param sp;
    sp.sched_priority = priority;
    const int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (ret != 0) {
        std::cerr << "[WARNING] " << label
                  << " thread realtime priority set failed: "
                  << std::strerror(ret)
                  << " (run as root or grant CAP_SYS_NICE)\n";
    } else {
        std::cout << "[System] " << label
                  << " thread realtime priority set (SCHED_FIFO, "
                  << priority << ")\n";
    }
}

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_REALTIME_UTILS_HPP
