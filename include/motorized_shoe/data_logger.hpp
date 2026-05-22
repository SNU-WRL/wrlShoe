#ifndef MOTORIZED_SHOE_DATA_LOGGER_HPP
#define MOTORIZED_SHOE_DATA_LOGGER_HPP

#include <fstream>
#include <string>
#include <vector>

#include "motorized_shoe/types.hpp"

namespace motorized_shoe {

class DataLogger {
public:
    explicit DataLogger(const std::string& path);
    ~DataLogger();

    void queue_snapshot(const SystemSnapshot& snapshot);
    void flush();

private:
    void write_row(const SystemSnapshot& snapshot);

    std::ofstream file_;
    int64_t start_time_ns_ = -1;
    std::vector<SystemSnapshot> pending_;
};

}  // namespace motorized_shoe

#endif  // MOTORIZED_SHOE_DATA_LOGGER_HPP
