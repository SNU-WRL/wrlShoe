#include "motorized_shoe/config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace motorized_shoe {
namespace {

struct StackEntry {
    int indent = 0;
    std::string key;
};

std::string trim(const std::string& in) {
    size_t start = 0;
    while (start < in.size() && std::isspace(static_cast<unsigned char>(in[start])) != 0) {
        ++start;
    }
    size_t end = in.size();
    while (end > start && std::isspace(static_cast<unsigned char>(in[end - 1])) != 0) {
        --end;
    }
    return in.substr(start, end - start);
}

std::string strip_quotes(const std::string& in) {
    if (in.size() >= 2 && ((in.front() == '"' && in.back() == '"') || (in.front() == '\'' && in.back() == '\''))) {
        return in.substr(1, in.size() - 2);
    }
    return in;
}

std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

int parse_int(const std::string& value) {
    return std::stoi(value, nullptr, 0);
}

float parse_float(const std::string& value) {
    return std::stof(value);
}

bool parse_bool(const std::string& value) {
    const std::string lowered = to_lower(value);
    return lowered == "true" || lowered == "1";
}

std::string join_path(const std::vector<StackEntry>& stack, const std::string& key) {
    std::string path;
    for (const auto& entry : stack) {
        if (!entry.key.empty()) {
            if (!path.empty()) {
                path += '.';
            }
            path += entry.key;
        }
    }
    if (!key.empty()) {
        if (!path.empty()) {
            path += '.';
        }
        path += key;
    }
    return path;
}

std::unordered_map<std::string, std::string> parse_yaml_key_values(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    std::unordered_map<std::string, std::string> kv;
    std::vector<StackEntry> stack;
    std::string line;

    while (std::getline(in, line)) {
        const size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        if (trim(line).empty()) {
            continue;
        }

        int indent = 0;
        while (indent < static_cast<int>(line.size()) && line[static_cast<size_t>(indent)] == ' ') {
            ++indent;
        }

        const std::string content = trim(line);
        const size_t colon = content.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        std::string key = trim(content.substr(0, colon));
        std::string value = trim(content.substr(colon + 1));

        while (!stack.empty() && indent <= stack.back().indent) {
            stack.pop_back();
        }

        if (value.empty()) {
            stack.push_back({indent, key});
            continue;
        }

        value = strip_quotes(value);
        const std::string path_key = join_path(stack, key);
        kv[path_key] = value;
    }

    return kv;
}

std::string find_value(const std::unordered_map<std::string, std::string>& kv, const std::string& suffix) {
    for (const auto& item : kv) {
        if (item.first == suffix) {
            return item.second;
        }
        if (item.first.size() > suffix.size() + 1 &&
            item.first.compare(item.first.size() - suffix.size(), suffix.size(), suffix) == 0) {
            return item.second;
        }
    }
    return "";
}

void apply_int(const std::unordered_map<std::string, std::string>& kv, const std::string& key, int& out) {
    const std::string value = find_value(kv, key);
    if (!value.empty()) {
        out = parse_int(value);
    }
}

void apply_float(const std::unordered_map<std::string, std::string>& kv, const std::string& key, float& out) {
    const std::string value = find_value(kv, key);
    if (!value.empty()) {
        out = parse_float(value);
    }
}

void apply_bool(const std::unordered_map<std::string, std::string>& kv, const std::string& key, bool& out) {
    const std::string value = find_value(kv, key);
    if (!value.empty()) {
        out = parse_bool(value);
    }
}

void apply_string(const std::unordered_map<std::string, std::string>& kv, const std::string& key, std::string& out) {
    const std::string value = find_value(kv, key);
    if (!value.empty()) {
        out = value;
    }
}

}  // namespace

Config load_config(const std::string& path) {
    Config cfg;
    const auto kv = parse_yaml_key_values(path);

    apply_string(kv, "can_imu_interface", cfg.can_imu_interface);
    apply_string(kv, "can_elmo_interface", cfg.can_elmo_interface);

    apply_int(kv, "imu_left_can_ids.rotation_vector", cfg.imu_left_can_ids.rotation_vector);
    apply_int(kv, "imu_left_can_ids.accelerometer", cfg.imu_left_can_ids.accelerometer);
    apply_int(kv, "imu_left_can_ids.gyroscope", cfg.imu_left_can_ids.gyroscope);
    apply_int(kv, "imu_left_can_ids.magnetometer", cfg.imu_left_can_ids.magnetometer);

    apply_int(kv, "imu_right_can_ids.rotation_vector", cfg.imu_right_can_ids.rotation_vector);
    apply_int(kv, "imu_right_can_ids.accelerometer", cfg.imu_right_can_ids.accelerometer);
    apply_int(kv, "imu_right_can_ids.gyroscope", cfg.imu_right_can_ids.gyroscope);
    apply_int(kv, "imu_right_can_ids.magnetometer", cfg.imu_right_can_ids.magnetometer);

    apply_int(kv, "elmo_node_ids.left", cfg.elmo_node_left);
    apply_int(kv, "elmo_node_ids.right", cfg.elmo_node_right);

    apply_int(kv, "loop_frequency", cfg.loop_frequency_hz);

    {
        int v = cfg.profile_acceleration;
        apply_int(kv, "elmo_config.profile_acceleration", v);
        cfg.profile_acceleration = static_cast<int32_t>(v);
    }
    {
        int v = cfg.profile_deceleration;
        apply_int(kv, "elmo_config.profile_deceleration", v);
        cfg.profile_deceleration = static_cast<int32_t>(v);
    }

    apply_float(kv, "gait_detection.sampling_frequency", cfg.gait_sampling_frequency);
    apply_bool(kv, "gait_detection.use_both_feet", cfg.gait_use_both_feet);

    apply_float(kv, "gait_detection.thresholds.hs_threshold", cfg.gait_thresholds.hs_threshold);
    apply_float(kv, "gait_detection.thresholds.ts_threshold", cfg.gait_thresholds.ts_threshold);
    apply_float(kv, "gait_detection.thresholds.ho_threshold", cfg.gait_thresholds.ho_threshold);
    apply_float(kv, "gait_detection.thresholds.to_threshold", cfg.gait_thresholds.to_threshold);
    apply_float(kv, "gait_detection.thresholds.swing_gyro_threshold", cfg.gait_thresholds.swing_gyro_threshold);
    apply_float(kv, "gait_detection.thresholds.midstance_threshold", cfg.gait_thresholds.midstance_threshold);
    apply_int(kv, "gait_detection.thresholds.min_swing_dwell_ms", cfg.gait_thresholds.min_swing_dwell_ms);

    apply_int(kv, "gait_detection.ma_window", cfg.gait_ma_window);
    apply_int(kv, "gait_detection.gravity_calib_samples", cfg.gravity_calib_samples);

    for (const char* phase : {"MSt", "HO", "TSt", "TO", "Swing", "HS"}) {
        const std::string key = std::string("gait_detection.velocity_map.") + phase;
        const std::string value = find_value(kv, key);
        if (!value.empty()) {
            cfg.velocity_map[phase] = parse_int(value);
        }
    }

    apply_bool(kv, "slip_perturbation.enabled", cfg.slip.enabled);
    apply_string(kv, "slip_perturbation.foot", cfg.slip.foot);
    {
        int v = cfg.slip.slip_velocity;
        apply_int(kv, "slip_perturbation.slip_velocity", v);
        cfg.slip.slip_velocity = static_cast<int32_t>(v);
    }
    apply_int(kv, "slip_perturbation.slip_duration_ms", cfg.slip.slip_duration_ms);
    apply_int(kv, "slip_perturbation.mode1_delay_after_hs_ms", cfg.slip.mode1_delay_after_hs_ms);
    apply_int(kv, "slip_perturbation.mode2_delay_after_mst_ms", cfg.slip.mode2_delay_after_mst_ms);
    {
        int v = cfg.slip.slip_profile_acceleration;
        apply_int(kv, "slip_perturbation.slip_profile_acceleration", v);
        cfg.slip.slip_profile_acceleration = static_cast<int32_t>(v);
    }
    {
        std::string s;
        apply_string(kv, "slip_perturbation.mode1_key", s);
        if (!s.empty()) cfg.slip.mode1_key = s.front();
        s.clear();
        apply_string(kv, "slip_perturbation.mode2_key", s);
        if (!s.empty()) cfg.slip.mode2_key = s.front();
    }

    apply_bool(kv, "logging.imu_data_log", cfg.log_imu);
    apply_bool(kv, "logging.gait_phase_log", cfg.log_gait);
    apply_bool(kv, "logging.elmo_command_log", cfg.log_command);
    apply_bool(kv, "logging.elmo_status_log", cfg.log_status);

    return cfg;
}

}  // namespace motorized_shoe
