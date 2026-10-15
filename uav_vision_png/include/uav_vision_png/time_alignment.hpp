#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <cstdint>
#include <Eigen/Geometry>

namespace vision_time {

// Bounded history of simultaneous Gazebo simulation/system clock samples.
// No receipt-derived offset: transport/inference latency is not clock offset.
class ClockHistory {
public:
    bool add(double sim, double system, double received)
    {
        if (!std::isfinite(sim) || sim < 0.0 || !std::isfinite(system) ||
            !std::isfinite(received) || system <= 0.0 ||
            received - system < -0.02 || received - system > 0.5) {
            return false;
        }
        bool reset = false;
        if (size_ != 0) {
            const auto &last = at(size_ - 1);
            if (sim < last.sim || system <= last.system) {
                clear();
                reset = true;
            } else if (sim == last.sim) {
                // A paused simulation has no invertible sim/system mapping.
                clear();
                reset = true;
            }
        }
        samples_[(begin_ + size_) % samples_.size()] = {sim, system};
        if (size_ == samples_.size()) {
            begin_ = (begin_ + 1) % samples_.size();
        } else {
            ++size_;
        }
        last_received_ = received;
        return reset;
    }

    bool map(double sim, double now, double &system) const
    {
        system = std::numeric_limits<double>::quiet_NaN();
        if (size_ < 2 || !std::isfinite(sim) || sim <= 0.0 ||
            now - last_received_ < 0.0 || now - last_received_ > 0.25 ||
            sim < at(0).sim || sim > at(size_ - 1).sim) {
            return false;
        }
        // Binary search in the logical ring; no copying or per-frame allocation.
        std::size_t low = 0, high = size_ - 1;
        while (low + 1 < high) {
            const auto mid = low + (high - low) / 2;
            if (at(mid).sim < sim) low = mid;
            else high = mid;
        }
        const auto &a = at(low);
        const auto &b = at(high);
        if (b.sim - a.sim > 0.1 || b.system - a.system > 0.2) return false;
        system = a.system + (sim - a.sim) / (b.sim - a.sim) * (b.system - a.system);
        if (now - system < 0.0 || now - system > 2.0) {
            system = std::numeric_limits<double>::quiet_NaN();
            return false;
        }
        return true;
    }

    void clear() { begin_ = size_ = 0; }

private:
    struct Sample { double sim = 0.0; double system = 0.0; };
    const Sample &at(std::size_t index) const { return samples_[(begin_ + index) % samples_.size()]; }
    std::array<Sample, 4096> samples_{};
    std::size_t begin_ = 0, size_ = 0;
    double last_received_ = 0.0;
};

// PX4 DDS has already translated timestamp_sample into the Agent system epoch.
// Never interpolate across estimator resets or large sample gaps.
class AttitudeHistory {
public:
    bool add(double sample, double received, const Eigen::Quaterniond &quaternion,
             std::uint8_t reset_counter)
    {
        const double norm_squared = quaternion.squaredNorm();
        if (!std::isfinite(sample) || sample <= 0.0 ||
            !std::isfinite(received) || received - sample < -0.02 ||
            received - sample > 0.5 || !quaternion.coeffs().allFinite() ||
            !std::isfinite(norm_squared) || norm_squared < 1e-12) return false;
        bool reset = false;
        if (size_ != 0) {
            if (reset_counter != reset_counter_ ||
                sample < at(size_ - 1).time - 0.1 || received < last_received_) {
                clear();
                reset = true;
            } else if (sample <= at(size_ - 1).time) {
                return false;  // Duplicate/reordered DDS samples cannot extend history.
            }
        }
        samples_[(begin_ + size_) % samples_.size()] =
            {sample, quaternion.normalized()};
        if (size_ == samples_.size()) begin_ = (begin_ + 1) % samples_.size();
        else ++size_;
        reset_counter_ = reset_counter;
        last_received_ = received;
        return reset;
    }

    bool interpolate(double time, Eigen::Quaterniond &quaternion,
                     double &before, double &after) const
    {
        before = after = std::numeric_limits<double>::quiet_NaN();
        if (size_ == 0 || !std::isfinite(time) ||
            time < at(0).time || time > at(size_ - 1).time) return false;
        std::size_t low = 0, high = size_ - 1;
        while (low + 1 < high) {
            const auto mid = low + (high - low) / 2;
            if (at(mid).time < time) low = mid;
            else high = mid;
        }
        const auto &a = at(low);
        const auto &b = at(high);
        if (time == a.time || time == b.time) {
            const auto &exact = time == a.time ? a : b;
            quaternion = exact.quaternion;
            before = after = exact.time;
            return true;
        }
        if (b.time - a.time > 0.1) return false;
        quaternion = a.quaternion.slerp((time - a.time) / (b.time - a.time),
                                       b.quaternion).normalized();
        before = a.time;
        after = b.time;
        return true;
    }

    double newest_time() const
    {
        return size_ == 0 ? std::numeric_limits<double>::quiet_NaN() : at(size_ - 1).time;
    }

    void clear() { begin_ = size_ = 0; }

private:
    struct Sample { double time; Eigen::Quaterniond quaternion; };
    const Sample &at(std::size_t index) const { return samples_[(begin_ + index) % samples_.size()]; }
    std::array<Sample, 512> samples_;
    std::size_t begin_ = 0, size_ = 0;
    std::uint8_t reset_counter_ = 0;
    double last_received_ = 0.0;
};

}  // namespace vision_time
