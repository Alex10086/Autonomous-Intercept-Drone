#include "uav_vision_png/pixel_kalman.hpp"
#include "uav_vision_png/time_alignment.hpp"
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}
}

int main()
{
    vision_time::PixelKalman filter;
    Eigen::Vector4d predicted;
    require(!filter.predict(10, 0.3, predicted), "uninitialized prediction");
    double time = 10;
    for (int i = 0; i < 40; ++i) {
        const double noise = i % 2 ? 0.3 : -0.3;
        const Eigen::Vector2d pixel(900 + 100 * (time - 10) + noise,
                                   500 - 60 * (time - 10) - noise);
        require(filter.update(time, pixel, 16, 1e6), "finite image update");
        time += i % 2 ? 0.07 : 0.03;
    }
    const double latest = time - 0.07;
    require(filter.predict(latest + 0.1, 0.3, predicted), "delayed current prediction");
    require(std::abs(predicted(0) - (900 + 100 * (latest + 0.1 - 10))) < 2,
            "horizontal delay compensation");
    require(std::abs(predicted(1) - (500 - 60 * (latest + 0.1 - 10))) < 2,
            "vertical delay compensation");
    require(std::abs(predicted(2) - 100) < 10 && std::abs(predicted(3) + 60) < 10,
            "learned independent pixel velocities");
    const auto unchanged = predicted;
    require(!filter.update(latest, Eigen::Vector2d(0, 0), 16, 1e6), "duplicate image accepted");
    require(!filter.update(latest - 0.01, Eigen::Vector2d(0, 0), 16, 1e6), "out-of-order image accepted");
    require(!filter.update(latest + 0.01,
        Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0), 16, 1e6), "NaN image accepted");
    require(filter.predict(latest + 0.1, 0.3, predicted) &&
            (predicted - unchanged).norm() < 1e-12, "rejected image poisoned state");
    require(!filter.predict(latest - 0.01, 0.3, predicted), "backward prediction");
    require(!filter.predict(latest + 0.31, 0.3, predicted), "unbounded prediction");
    require(filter.update(latest + 0.6, Eigen::Vector2d(400, 300), 16, 1e6), "reacquisition update");
    require(filter.predict(latest + 0.7, 0.3, predicted) &&
            (predicted - Eigen::Vector4d(400, 300, 0, 0)).norm() < 1e-10,
            "velocity carried across a blind gap");
    filter.reset();
    require(!filter.predict(latest + 0.7, 0.3, predicted), "reset state remained usable");

    const Eigen::Quaterniond attitude(Eigen::AngleAxisd(std::acos(-1.0) / 2,
                                                       Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rotation;
    require(vision_time::propagate_attitude(attitude, Eigen::Vector3d::UnitX(), 0.02, rotation),
            "short gyro propagation");
    const Eigen::Matrix3d expected = attitude.toRotationMatrix() *
        Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX()).toRotationMatrix();
    require((rotation - expected).norm() < 1e-12, "body/world rotation order mixed");
    require(!vision_time::propagate_attitude(attitude, Eigen::Vector3d::Zero(), 0.051, rotation),
            "stale attitude extrapolated");
    require(!vision_time::propagate_attitude(attitude, Eigen::Vector3d::Zero(), -0.001, rotation),
            "future attitude extrapolated backwards");
    std::cout << "Pixel KF: irregular image epochs, delayed prediction, loss/reset and body-rate propagation passed\n";
}
