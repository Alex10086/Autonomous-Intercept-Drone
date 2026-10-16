#include "rotation_prior.hpp"
#include "tracking_validation.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using vision_tracking::CameraIntrinsics;
using vision_tracking::rotate_camera_prior;

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

CameraIntrinsics camera()
{
    return {{{1400.0, 0.0, 960.0, 0.0, 1400.0, 540.0, 0.0, 0.0, 1.0}}, 1920, 1080};
}

Eigen::Quaterniond rotation(double angle, const Eigen::Vector3d& axis)
{
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

void require_near_box(const cv::Rect& actual, const cv::Rect& expected, const char* message)
{
    require(std::abs(actual.x - expected.x) <= 1 && std::abs(actual.y - expected.y) <= 1 &&
        std::abs(actual.br().x - expected.br().x) <= 1 &&
        std::abs(actual.br().y - expected.br().y) <= 1, message);
}

void test_geometry()
{
    const auto intrinsics = camera();
    const auto identity = Eigen::Quaterniond::Identity();
    const cv::Rect anchor(1280, 496, 55, 39);
    cv::Rect prior;
    require(rotate_camera_prior(anchor, intrinsics, identity, identity, prior),
        "identity transform rejected");
    require_near_box(prior, anchor, "identity rotation moved the prior");
    const auto pitched = rotation(-0.25, Eigen::Vector3d::UnitY());
    require(rotate_camera_prior(anchor, intrinsics, identity, pitched, prior),
        "valid pitch rotation rejected");
    require(anchor.y - prior.y > 340 && anchor.y - prior.y < 380,
        "quarter-radian pitch did not produce the observed hundreds-of-pixels displacement");
    const cv::Rect visible_target(1298, 137, 55, 39);
    require(vision_tracking::intersection_over_union(anchor, visible_target) == 0.0 &&
        vision_tracking::intersection_over_union(prior, visible_target) > 0.5,
        "rotated prior did not match a displaced target rejected by the old pixel anchor");

    // Independent scalar yaw-then-roll inverse applied to the pinhole rays.
    // This also detects camera/body axis permutations and reversed products.
    const double yaw = 0.14, roll = -0.19;
    const auto current = rotation(yaw, Eigen::Vector3d::UnitZ()) *
        rotation(roll, Eigen::Vector3d::UnitX());
    require(rotate_camera_prior(anchor, intrinsics, identity, current, prior),
        "combined yaw/roll rejected");
    double left = 1920.0, top = 1080.0, right = 0.0, bottom = 0.0;
    for (const int u : {anchor.x, anchor.br().x}) {
        for (const int v : {anchor.y, anchor.br().y}) {
            const double x = (u - 960.0) / 1400.0, y = (v - 540.0) / 1400.0;
            const double forward = std::cos(yaw) + std::sin(yaw) * x;
            const double lateral = -std::sin(yaw) + std::cos(yaw) * x;
            const double horizontal = std::cos(roll) * lateral + std::sin(roll) * y;
            const double vertical = -std::sin(roll) * lateral + std::cos(roll) * y;
            const double projected_u = 960.0 + 1400.0 * horizontal / forward;
            const double projected_v = 540.0 + 1400.0 * vertical / forward;
            left = std::min(left, projected_u);
            right = std::max(right, projected_u);
            top = std::min(top, projected_v);
            bottom = std::max(bottom, projected_v);
        }
    }
    const cv::Rect expected(static_cast<int>(std::floor(left)), static_cast<int>(std::floor(top)),
        static_cast<int>(std::ceil(right)) - static_cast<int>(std::floor(left)),
        static_cast<int>(std::ceil(bottom)) - static_cast<int>(std::floor(top)));
    require_near_box(prior, expected, "yaw/roll multiplication or body-camera axes reversed");
    cv::Rect reversed;
    require(rotate_camera_prior(anchor, intrinsics, identity,
        rotation(roll, Eigen::Vector3d::UnitX()) * rotation(yaw, Eigen::Vector3d::UnitZ()), reversed) &&
        std::abs(prior.y - reversed.y) > 20, "test did not distinguish noncommuting yaw/roll order");
    require(rotate_camera_prior(anchor, intrinsics, current, current, prior),
        "equal nonidentity attitudes rejected");
    require_near_box(prior, anchor, "anchor attitude was not cancelled against current attitude");

    require(!rotate_camera_prior(anchor, intrinsics, identity,
        rotation(std::acos(-1.0), Eigen::Vector3d::UnitZ()), prior) && prior.width < 0,
        "behind-camera target retained a valid prior");
    require(!rotate_camera_prior(anchor, intrinsics, identity,
        rotation(1.0, Eigen::Vector3d::UnitZ()), prior) && prior.width < 0,
        "out-of-image target fell back to the old coordinates");
    auto invalid = intrinsics;
    invalid.k[0] = 0.0;
    require(!rotate_camera_prior(anchor, invalid, identity, identity, prior),
        "zero focal length accepted");
    invalid = intrinsics;
    invalid.k[4] = std::numeric_limits<double>::quiet_NaN();
    require(!rotate_camera_prior(anchor, invalid, identity, identity, prior),
        "nonfinite calibration accepted");
    invalid = intrinsics;
    invalid.k[8] = 0.0;
    require(!rotate_camera_prior(anchor, invalid, identity, identity, prior),
        "non-pinhole calibration accepted");
    invalid = intrinsics;
    invalid.width = 0;
    require(!rotate_camera_prior(anchor, invalid, identity, identity, prior),
        "missing image dimensions accepted");
    require(!rotate_camera_prior(anchor, intrinsics, Eigen::Quaterniond(0, 0, 0, 0), identity, prior),
        "invalid attitude accepted");
}

void test_bounded_support()
{
    using namespace vision_tracking;
    struct Candidate { cv::Rect box; float conf; };
    const cv::Rect anchor(1280, 496, 55, 39), prediction(1168, 361, 95, 74);
    const cv::Rect image(0, 0, 1920, 1080);
    cv::Rect prior;
    require(rotate_camera_prior(anchor, camera(), Eigen::Quaterniond::Identity(),
        rotation(-0.25, Eigen::Vector3d::UnitY()), prior), "support prior unavailable");
    const std::vector<Candidate> candidates{{{1298, 137, 55, 39}, 0.776317f}};
    const auto at = [](int ms) { return ContinuityBudget::Time{} + std::chrono::milliseconds(ms); };
    ContinuityBudget budget(0.35, 2.0);
    budget.acquire(at(0));
    budget.begin_frame(at(248));
    budget.miss();
    require(select_tracking_detection(candidates, image, prediction, anchor, budget, at(248),
        0.1, 0.8f).action == DetectionAction::Invalid, "old anchor unexpectedly authorized support");
    const auto recovered = select_tracking_detection(candidates, image, prediction, prior,
        budget, at(248), 0.1, 0.8f);
    require(recovered.action == DetectionAction::Acquire && recovered.local_recovery,
        "rotated anchor did not recover actual low-score detector evidence");
    require(!budget.publication_allowed(at(351)) && budget.in_probation(),
        "computing or matching a prior itself renewed permission");
    require(select_detection(candidates, image, nullptr, 0.1, 0.8f).action == DetectionAction::Invalid,
        "rotation lowered the global acquisition threshold");
    require(select_tracking_detection(candidates, image, prediction, prior, budget, at(2001),
        0.1, 0.8f).action == DetectionAction::Invalid, "rotation bypassed the recovery deadline");
    const std::vector<Candidate> preferred{{prediction, 0.6f}, candidates.front()};
    const auto supported = select_tracking_detection(preferred, image, prediction, prior,
        budget, at(248), 0.1, 0.8f);
    require(supported.candidate_index == 0 && !supported.local_recovery,
        "rotation displaced currently supported tracker priority");
    budget.support(at(248));
    require(select_tracking_detection(candidates, image, prediction, prior, budget, at(300),
        0.1, 0.8f).action == DetectionAction::Invalid, "rotation enabled fallback outside probation");
}

void test_capture_history()
{
    vision_tracking::CaptureRotationHistory history;
    const auto first = Eigen::Quaterniond::Identity();
    const auto second = rotation(0.2, Eigen::Vector3d::UnitY());
    require(!history.sample(10.02, 1000.04).valid, "missing timing invented an attitude");
    history.clock(10.0, 1000.0, 1000.0);
    history.clock(10.02, 1000.02, 1000.02);
    history.timesync(1000000000ULL, 1000000000ULL, -990000000LL, 0, 1000.02);
    history.odometry(1000010000ULL, first, 0, 1000.02);
    require(!history.sample(10.02, 1000.02).valid, "latest receipt pose replaced missing bracket");
    // Sample .03 arrives before the .04 native clock pair: it must be queued,
    // then flushed when the simultaneous clocks bracket it.
    history.odometry(1000030000ULL, second, 0, 1000.035);
    require(!history.sample(10.02, 1000.035).valid, "unmapped pending attitude was used");
    history.clock(10.04, 1000.04, 1000.04);
    const auto interpolated = history.sample(10.02, 1000.04);
    require(interpolated.valid && std::abs(interpolated.epoch - 1000.02) < 1e-6 &&
        interpolated.before < interpolated.epoch && interpolated.after > interpolated.epoch &&
        interpolated.b2n.angularDistance(rotation(0.1, Eigen::Vector3d::UnitY())) < 1e-5,
        "capture sample did not SLERP mapped native odometry timestamps");
    require(!history.sample(10.039, 1000.04).valid,
        "unbracketed frame fell back to the newest attitude");
    history.odometry(1000035000ULL, second, 1, 1000.04);
    const auto estimator_reset = history.sample(10.02, 1000.04);
    require(!estimator_reset.valid && estimator_reset.generation != interpolated.generation,
        "estimator reset retained old attitude generation");
    history.timesync(1000000000ULL, 1000040000ULL, -989970000LL, 0, 1000.04);
    const auto offset_reset = history.sample(10.02, 1000.04);
    require(!offset_reset.valid && offset_reset.generation != estimator_reset.generation,
        "DDS offset step retained old attitude generation");
    history.clock(1.0, 1000.05, 1000.05);
    const auto clock_reset = history.sample(1.0, 1000.05);
    require(!clock_reset.valid && clock_reset.generation != offset_reset.generation,
        "simulation clock reset retained old attitude generation");
}
}

int main()
{
    test_geometry();
    test_bounded_support();
    test_capture_history();
    std::cout << "rotation prior behavior passed\n";
    return 0;
}
