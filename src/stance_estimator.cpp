#include "motorized_shoe/stance_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace motorized_shoe {

namespace {
template <typename T>
T median_of(std::deque<T> v) {
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return (n % 2 == 1) ? v[n / 2] : static_cast<T>((v[n / 2 - 1] + v[n / 2]) / 2);
}
int64_t ms_to_ns(int ms) { return static_cast<int64_t>(ms) * 1000000LL; }
}  // namespace

StanceEstimator::StanceEstimator(const StanceEstimatorConfig& cfg) : cfg_(cfg) {
    if (cfg_.window < 1) cfg_.window = 1;
    if (cfg_.warmup_cycles < 1) cfg_.warmup_cycles = 1;
}

void StanceEstimator::reset(const char* reason) {
    stances_ns_.clear();
    fractions_.clear();
    periods_ns_.clear();
    have_last_hs_ = false;
    stance_open_ = false;
    open_stance_ns_ = -1;
    last_cycle_ns_ = 0;
    last_reset_reason_ = reason ? reason : "";
}

std::string StanceEstimator::on_heel_strike(int64_t t_hs_ns) {
    std::ostringstream note;
    if (have_last_hs_) {
        const int64_t period = t_hs_ns - last_hs_ns_;
        if (period > ms_to_ns(cfg_.reset_gap_ms) || period <= 0) {
            reset("HS gap");
            note << "estimator reset: HS gap " << (period / 1000000) << " ms";
        } else {
            const bool period_ok =
                period >= ms_to_ns(cfg_.cycle_min_ms) && period <= ms_to_ns(cfg_.cycle_max_ms);
            if (!period_ok) {
                note << "cycle " << (period / 1000000) << " ms outside ["
                     << cfg_.cycle_min_ms << "," << cfg_.cycle_max_ms << "] rejected";
                // The stance measured inside an implausible cycle is not
                // trusted either.
                if (open_stance_ns_ > 0 && !stances_ns_.empty() &&
                    stances_ns_.back() == open_stance_ns_) {
                    stances_ns_.pop_back();
                }
            } else {
                last_cycle_ns_ = period;
                periods_ns_.push_back(period);
                while (static_cast<int>(periods_ns_.size()) > cfg_.window) {
                    periods_ns_.pop_front();
                }
                if (open_stance_ns_ > 0) {
                    fractions_.push_back(static_cast<double>(open_stance_ns_) /
                                         static_cast<double>(period));
                    while (static_cast<int>(fractions_.size()) > cfg_.window) {
                        fractions_.pop_front();
                    }
                }
            }
        }
    }
    have_last_hs_ = true;
    last_hs_ns_ = t_hs_ns;
    stance_open_ = true;
    open_stance_ns_ = -1;
    return note.str();
}

std::string StanceEstimator::on_toe_off(int64_t t_to_ns) {
    if (!stance_open_) {
        return "";
    }
    stance_open_ = false;
    const int64_t stance = t_to_ns - last_hs_ns_;
    if (stance < ms_to_ns(cfg_.stance_min_ms) || stance > ms_to_ns(cfg_.stance_max_ms)) {
        open_stance_ns_ = -1;
        std::ostringstream note;
        note << "stance " << (stance / 1000000) << " ms outside [" << cfg_.stance_min_ms << ","
             << cfg_.stance_max_ms << "] rejected";
        return note.str();
    }
    open_stance_ns_ = stance;
    stances_ns_.push_back(stance);
    while (static_cast<int>(stances_ns_.size()) > cfg_.window) {
        stances_ns_.pop_front();
    }
    return "";
}

bool StanceEstimator::warm() const {
    return static_cast<int>(stances_ns_.size()) >= cfg_.warmup_cycles;
}

int64_t StanceEstimator::cycle_for_prediction_ns() const {
    if (periods_ns_.empty() || last_cycle_ns_ <= 0) {
        return last_cycle_ns_;
    }
    const int64_t med = median_of(periods_ns_);
    const double dev = std::abs(static_cast<double>(last_cycle_ns_ - med)) / static_cast<double>(med);
    return (dev * 100.0 > static_cast<double>(cfg_.cadence_change_pct)) ? last_cycle_ns_ : med;
}

int64_t StanceEstimator::predict_stance_ns() const {
    if (!warm()) {
        return 0;
    }
    const int64_t cycle = cycle_for_prediction_ns();
    if (!fractions_.empty() && cycle > 0) {
        return static_cast<int64_t>(median_of(fractions_) * static_cast<double>(cycle));
    }
    return median_of(stances_ns_);
}

std::string StanceEstimator::describe() const {
    std::ostringstream s;
    s << "clean stances " << stances_ns_.size() << "/" << cfg_.warmup_cycles << " (window "
      << cfg_.window << ")";
    if (!fractions_.empty()) {
        s << ", median fraction " << median_of(fractions_);
    }
    if (last_cycle_ns_ > 0) {
        s << ", last cycle " << (last_cycle_ns_ / 1000000) << " ms";
    }
    if (warm()) {
        s << ", predicted stance " << (predict_stance_ns() / 1000000) << " ms";
    } else {
        s << ", NOT warm";
    }
    if (!last_reset_reason_.empty()) {
        s << ", last reset: " << last_reset_reason_;
    }
    return s.str();
}

}  // namespace motorized_shoe
