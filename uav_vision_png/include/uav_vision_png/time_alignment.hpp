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
        if (size_ < 2 || !std::isfinite(sim) || sim <= 0.0 || !std::isfinite(now) ||
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

// Recover native Gazebo/PX4 HRT before using the simultaneous native clock pairs.
// PX4 GZBridge::clockCallback installs Clock.sim as CLOCK_MONOTONIC; drv_hrt.cpp
// returns that lockstep absolute time. uXRCE-DDS serialization applies
// session.time_offset = -Timesync.estimated_offset to BOTH timestamp fields:
//     DDS = HRT - estimated_offset; HRT = DDS + estimated_offset.
// Receipt time is only an epoch/freshness guard, never an offset measurement.
class DdsClock {
public:
    // Invalid input leaves the last usable status untouched. An accepted epoch
    // reversal or >20ms offset step replaces it and asks the caller to discard
    // pose/queued history, whose DDS samples may use the previous offset.
    bool update(std::uint64_t timestamp, std::uint64_t remote_timestamp,
                std::int64_t estimated_offset, std::uint32_t round_trip_time,
                double received)
    {
        std::int64_t sim;
        if (!std::isfinite(received) || received <= 0.0 ||
            remote_timestamp == 0 ||
            remote_timestamp > static_cast<std::uint64_t>(
                std::numeric_limits<std::int64_t>::max()) ||
            !recover(timestamp, estimated_offset, sim) ||
            std::abs(static_cast<double>(remote_timestamp) * 1e-6 - received) > 2.0) {
            return false;
        }
        const auto delta = estimated_offset >= offset_
            ? static_cast<std::uint64_t>(estimated_offset) - static_cast<std::uint64_t>(offset_)
            : static_cast<std::uint64_t>(offset_) - static_cast<std::uint64_t>(estimated_offset);
        const bool reset = available_ &&
            (delta > 20000 || remote_timestamp < remote_ || received < received_);
        sim_ = sim;
        remote_ = remote_timestamp;
        offset_ = estimated_offset;
        round_trip_time_ = round_trip_time;
        received_ = received;
        available_ = true;
        validated_ = false;
        return reset;
    }

    bool map(std::uint64_t sample, double now, const ClockHistory &clocks,
             double &system)
    {
        system = std::numeric_limits<double>::quiet_NaN();
        if (!available_ || !std::isfinite(now) ||
            now - received_ < 0.0 || now - received_ > 2.0) return false;
        std::int64_t sim;
        if (!recover(sample, offset_, sim)) return false;
        if (!validated_) {
            double status_system;
            if (!clocks.map(static_cast<double>(sim_) * 1e-6, now, status_system) ||
                std::abs(status_system - static_cast<double>(remote_) * 1e-6) >
                    0.05 + static_cast<double>(round_trip_time_) * 0.5e-6) {
                return false;
            }
            // Cache successes only: a status can arrive before its native pair.
            validated_ = true;
        }
        return clocks.map(static_cast<double>(sim) * 1e-6, now, system);
    }

    void clear()
    {
        available_ = validated_ = false;
    }

private:
    static bool recover(std::uint64_t timestamp, std::int64_t offset,
                        std::int64_t &sim)
    {
        if (timestamp == 0 ||
            timestamp > static_cast<std::uint64_t>(
                std::numeric_limits<std::int64_t>::max())) return false;
        const auto raw = static_cast<std::int64_t>(timestamp);
        if ((offset > 0 && raw > std::numeric_limits<std::int64_t>::max() - offset) ||
            (offset < 0 && offset < -raw)) return false;
        // Add integer microseconds first: subtracting two wall-epoch doubles
        // would lose the precision needed to recover the original HRT.
        sim = raw + offset;
        return sim > 0;
    }

    std::int64_t sim_ = 0, offset_ = 0;
    std::uint64_t remote_ = 0;
    std::uint32_t round_trip_time_ = 0;
    double received_ = 0.0;
    bool available_ = false, validated_ = false;
};

// Attitude times have already been mapped into the system capture-time epoch.
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

// Short current-epoch attitude propagation, not a control-law change.
inline bool propagate_attitude(const Eigen::Quaterniond &attitude,
                               const Eigen::Vector3d &body_rates, double dt,
                               Eigen::Matrix3d &rotation)
{
    const double norm_squared = attitude.squaredNorm();
    if (!std::isfinite(dt) || dt < 0.0 || dt > 0.05 ||
        !attitude.coeffs().allFinite() || !body_rates.allFinite() ||
        !std::isfinite(norm_squared) || norm_squared < 1e-12) return false;
    Eigen::Quaterniond predicted = attitude.normalized();
    const double speed = body_rates.norm();
    if (!std::isfinite(speed)) return false;
    if (speed > 1e-12) {
        predicted *= Eigen::Quaterniond(Eigen::AngleAxisd(speed * dt, body_rates / speed));
    }
    rotation = predicted.toRotationMatrix();
    return rotation.allFinite();
}

}  // namespace vision_time
