#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace vision_time {

inline bool project_pixel(const Eigen::Vector3d &ray, double focal,
                          const Eigen::Vector2d &principal, Eigen::Vector2d &pixel)
{
    if (!ray.allFinite() || ray.z() <= 1e-6) return false;
    pixel = principal + focal * ray.head<2>() / ray.z();
    return pixel.allFinite();
}

inline bool project_pixel_velocity(const Eigen::Vector3d &ray,
                                   const Eigen::Vector3d &ray_rate, double focal,
                                   Eigen::Vector2d &velocity)
{
    if (!ray.allFinite() || !ray_rate.allFinite() || ray.z() <= 1e-6) return false;
    velocity = focal * (ray_rate.head<2>() * ray.z() - ray.head<2>() * ray_rate.z()) /
        (ray.z() * ray.z());
    return velocity.allFinite();
}

// Fixed-plane pixel state at the last image epoch: [u, v, du/dt, dv/dt].
class PixelKalman {
public:
    void reset() { initialized_ = false; }

    bool update(double time, const Eigen::Vector2d &pixel,
                double measurement_variance, double acceleration_variance)
    {
        if (!std::isfinite(time) || !pixel.allFinite() ||
            !std::isfinite(measurement_variance) || measurement_variance <= 0.0 ||
            !std::isfinite(acceleration_variance) || acceleration_variance < 0.0 ||
            (initialized_ && time <= time_)) return false;
        if (!initialized_ || time - time_ > 0.5) {
            state_ << pixel.x(), pixel.y(), 0.0, 0.0;
            covariance_.setZero();
            covariance_.diagonal() << measurement_variance, measurement_variance, 1e6, 1e6;
            time_ = time;
            initialized_ = true;
            return true;
        }
        const double dt = time - time_, dt2 = dt * dt;
        Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
        transition(0, 2) = transition(1, 3) = dt;
        Eigen::Matrix4d noise = Eigen::Matrix4d::Zero();
        for (int axis = 0; axis < 2; ++axis) {
            noise(axis, axis) = acceleration_variance * dt2 * dt2 / 4.0;
            noise(axis, axis + 2) = noise(axis + 2, axis) = acceleration_variance * dt2 * dt / 2.0;
            noise(axis + 2, axis + 2) = acceleration_variance * dt2;
        }
        const Eigen::Vector4d prior_state = transition * state_;
        const Eigen::Matrix4d prior = transition * covariance_ * transition.transpose() + noise;
        const Eigen::Matrix2d innovation = prior.topLeftCorner<2, 2>() +
            measurement_variance * Eigen::Matrix2d::Identity();
        const double determinant = innovation.determinant();
        if (!std::isfinite(determinant) || determinant <= 0.0) return false;
        const Eigen::Matrix<double, 4, 2> gain = prior.leftCols<2>() * innovation.inverse();
        const Eigen::Vector4d next = prior_state + gain * (pixel - prior_state.head<2>());
        Eigen::Matrix4d residual = Eigen::Matrix4d::Identity();
        residual.leftCols<2>() -= gain;
        // Joseph form preserves covariance under many delayed, uneven-rate updates.
        const Eigen::Matrix4d covariance = residual * prior * residual.transpose() +
            measurement_variance * gain * gain.transpose();
        if (!next.allFinite() || !covariance.allFinite()) return false;
        state_ = next;
        covariance_ = covariance;
        time_ = time;
        return true;
    }

    bool predict(double time, double max_horizon, Eigen::Vector4d &state) const
    {
        state.setConstant(std::numeric_limits<double>::quiet_NaN());
        if (!initialized_ || !std::isfinite(time) || !std::isfinite(max_horizon) ||
            max_horizon <= 0.0 || time < time_ || time - time_ > max_horizon) return false;
        state = state_;
        state.head<2>() += (time - time_) * state_.tail<2>();
        return state.allFinite();
    }

private:
    bool initialized_ = false;
    double time_ = 0.0;
    Eigen::Vector4d state_;
    Eigen::Matrix4d covariance_;
};

}  // namespace vision_time
