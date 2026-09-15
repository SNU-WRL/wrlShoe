#ifndef MOTORIZED_SHOE_DATA_LOGGER_HPP
#define MOTORIZED_SHOE_DATA_LOGGER_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "motorized_shoe/types.hpp"

namespace motorized_shoe {

// CSV logger with a dedicated writer thread.
//
// The control thread only copies snapshots into a local batch
// (queue_snapshot) and hands the batch over under a mutex (flush). All row
// formatting and file I/O happen on the writer thread, which runs at normal
// (SCHED_OTHER) priority. Before 2026-09-15 the file write ran on the 1 kHz
// control thread and SD-card stalls blocked the loop for 10-95 ms (visible
// as simultaneous "IMU stalls" of both feet and as log_latency_us spikes).
//
// The queue is bounded (kMaxQueued snapshots, ~5 s at 1 kHz): if the card
// stalls longer than that, further snapshots are dropped and counted in the
// log_dropped column rather than growing memory without limit.
class DataLogger {
public:
    explicit DataLogger(const std::string& path);
    ~DataLogger();

    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    // Control thread: copy the snapshot into the current batch. Stamps
    // snapshot.log_queue_depth / log_dropped on the stored copy.
    void queue_snapshot(const SystemSnapshot& snapshot);
    // Control thread: hand the current batch to the writer (mutex + swap,
    // microseconds). Does not block on I/O.
    void flush();
    // Any thread: hand over the current batch and wait until every queued row
    // is on disk. Called by the destructor; safe to call more than once.
    void drain();

    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    void writer_loop();
    void write_row(const SystemSnapshot& snapshot);

    static constexpr size_t kMaxQueued = 5000;

    std::ofstream file_;
    int64_t start_time_ns_ = -1;

    // Control-thread side (no lock needed).
    std::vector<SystemSnapshot> pending_;

    // Shared with the writer.
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<SystemSnapshot> shared_;   // batches handed over, not yet taken
    bool stop_ = false;
    bool writer_busy_ = false;             // writer holds a batch it has not finished
    std::condition_variable idle_cv_;      // signalled when shared_ is empty and writer idle

    // Snapshots queued but not yet written (control batch + shared + in-flight).
    std::atomic<uint32_t> unwritten_{0};
    std::atomic<uint32_t> dropped_{0};
    bool drop_reported_ = false;

    std::thread writer_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_DATA_LOGGER_HPP
