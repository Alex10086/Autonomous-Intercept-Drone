#pragma once

#include <uav_vision_png/time_alignment.hpp>
#include <opencv2/core/types.hpp>
#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace vision_tracking {

struct CameraIntrinsics {
    std::array<double, 9> k{};
    int width = 0;
    int height = 0;

    bool operator==(const CameraIntrinsics& other) const
    {
        return width == other.width && height == other.height && k == other.k;
    }
};

// Pure camera rotation: no target velocity, vehicle translation, or ROI growth.
// Failure is authoritative when a known transform points behind/outside the image.
inline bool rotate_camera_prior(const cv::Rect& anchor, const CameraIntrinsics& camera,
    const Eigen::Quaterniond& anchor_b2n, const Eigen::Quaterniond& current_b2n,
    cv::Rect& prior)
{
    prior = cv::Rect(-1, -1, -1, -1);
    if (anchor.width <= 0 || anchor.height <= 0 || camera.width <= 0 || camera.height <= 0)
        return false;
    Eigen::Matrix3d k;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) k(row, col) = camera.k[row * 3 + col];
    if (!k.allFinite() || k(0, 0) <= 0.0 || k(1, 1) <= 0.0 ||
        k(2, 0) != 0.0 || k(2, 1) != 0.0 || k(2, 2) != 1.0 ||
        !std::isfinite(k.determinant()) || std::abs(k.determinant()) < 1e-12 ||
        !anchor_b2n.coeffs().allFinite() || !current_b2n.coeffs().allFinite() ||
        !std::isfinite(anchor_b2n.squaredNorm()) || !std::isfinite(current_b2n.squaredNorm()) ||
        anchor_b2n.squaredNorm() < 1e-12 || current_b2n.squaredNorm() < 1e-12) return false;
    Eigen::Matrix3d c2b;
    c2b << 0, 0, 1, 1, 0, 0, 0, 1, 0;
    const Eigen::Matrix3d rotation = c2b.transpose() *
        current_b2n.normalized().toRotationMatrix().transpose() *
        anchor_b2n.normalized().toRotationMatrix() * c2b;
    const Eigen::Matrix3d inverse_k = k.inverse();
    double left = std::numeric_limits<double>::infinity();
    double top = left, right = -left, bottom = -left;
    for (const double x : {static_cast<double>(anchor.x),
             static_cast<double>(anchor.x) + anchor.width}) {
        for (const double y : {static_cast<double>(anchor.y),
                 static_cast<double>(anchor.y) + anchor.height}) {
            const Eigen::Vector3d ray = rotation * inverse_k * Eigen::Vector3d(x, y, 1.0);
            if (!ray.allFinite() || ray.z() <= 0.0) return false;
            const Eigen::Vector3d pixel = k * ray;
            const double u = pixel.x() / pixel.z(), v = pixel.y() / pixel.z();
            if (!std::isfinite(u) || !std::isfinite(v)) return false;
            left = std::min(left, u);
            right = std::max(right, u);
            top = std::min(top, v);
            bottom = std::max(bottom, v);
        }
    }
    // Clip doubles before integer conversion, including near-horizon projections.
    left = std::max(0.0, left);
    top = std::max(0.0, top);
    right = std::min(static_cast<double>(camera.width), right);
    bottom = std::min(static_cast<double>(camera.height), bottom);
    if (right <= left || bottom <= top) return false;
    const int x = static_cast<int>(std::floor(left));
    const int y = static_cast<int>(std::floor(top));
    prior = cv::Rect(x, y, static_cast<int>(std::ceil(right)) - x,
        static_cast<int>(std::ceil(bottom)) - y);
    return true;
}

struct CaptureAttitude {
    Eigen::Quaterniond b2n = Eigen::Quaterniond::Identity();
    std::uint64_t generation = 0;
    double epoch = std::numeric_limits<double>::quiet_NaN();
    double before = std::numeric_limits<double>::quiet_NaN();
    double after = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;
};

// All methods are called under the node's short state mutex. Neither DDS receipt
// attitude nor extrapolation can substitute for an image-time SLERP bracket.
class CaptureRotationHistory {
public:
    void clock(double sim, double system, double received)
    {
        if (clocks_.add(sim, system, received)) {
            dds_.clear();
            reset();
        }
        flush(received);
    }

    void timesync(std::uint64_t timestamp, std::uint64_t remote, std::int64_t offset,
        std::uint32_t round_trip, double received)
    {
        if (dds_.update(timestamp, remote, offset, round_trip, received)) reset();
        flush(received);
    }

    void odometry(std::uint64_t sample, const Eigen::Quaterniond& b2n,
        std::uint8_t reset_counter, double received)
    {
        if (have_reset_counter_ && reset_counter != reset_counter_) reset();
        reset_counter_ = reset_counter;
        have_reset_counter_ = true;
        if (pending_size_ == pending_.size()) {
            for (std::size_t i = 1; i < pending_size_; ++i) pending_[i - 1] = pending_[i];
            --pending_size_;
        }
        pending_[pending_size_++] = {sample, b2n, reset_counter, received};
        flush(received);
    }

    CaptureAttitude sample(double image_sim, double now) const
    {
        CaptureAttitude result;
        result.generation = generation_;
        result.valid = clocks_.map(image_sim, now, result.epoch) &&
            attitudes_.interpolate(result.epoch, result.b2n, result.before, result.after);
        return result;
    }

private:
    struct Pending {
        std::uint64_t sample = 0;
        Eigen::Quaterniond b2n = Eigen::Quaterniond::Identity();
        std::uint8_t reset_counter = 0;
        double received = 0.0;
    };

    void reset()
    {
        attitudes_.clear();
        pending_size_ = 0;
        have_reset_counter_ = false;
        ++generation_;
    }

    void flush(double now)
    {
        std::size_t remaining = 0;
        for (std::size_t i = 0; i < pending_size_; ++i) {
            const auto& pending = pending_[i];
            if (now < pending.received || now - pending.received > 0.5) continue;
            double epoch;
            if (!dds_.map(pending.sample, now, clocks_, epoch)) {
                pending_[remaining++] = pending;
                continue;
            }
            if (attitudes_.add(epoch, pending.received, pending.b2n, pending.reset_counter)) {
                // The newly inserted sample belongs to the new epoch, but all
                // other queued samples and the old anchor belong to the old one.
                ++generation_;
                pending_size_ = 0;
                return;
            }
        }
        pending_size_ = remaining;
    }

    vision_time::ClockHistory clocks_;
    vision_time::DdsClock dds_;
    vision_time::AttitudeHistory attitudes_;
    std::array<Pending, 16> pending_{};
    std::size_t pending_size_ = 0;
    std::uint64_t generation_ = 0;
    std::uint8_t reset_counter_ = 0;
    bool have_reset_counter_ = false;
};

}  // namespace vision_tracking
