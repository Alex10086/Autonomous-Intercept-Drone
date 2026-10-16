#include "tracking_validation.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
struct Candidate {
    cv::Rect box;
    float conf;
};

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
}

int main()
{
    using vision_tracking::DetectionAction;
    using vision_tracking::select_detection;
    const cv::Rect image(0, 0, 1920, 1080);
    const cv::Rect tracked(100, 100, 40, 40);
    std::vector<Candidate> candidates{{{900, 500, 40, 40}, 0.99f}, {tracked, 0.6f}};
    auto decision = select_detection(candidates, image, &tracked, 0.1, 0.8f);
    require(decision.action == DetectionAction::KeepTracker && decision.candidate_index == 1,
        "unrelated high score stole low-score supported tracker");
    require(candidates[0].conf == 0.99f && candidates[1].conf == 0.6f,
        "support selection reordered the capture candidate array");

    candidates.push_back({{102, 102, 40, 40}, 0.85f});
    decision = select_detection(candidates, image, &tracked, 0.1, 0.8f);
    require(decision.action == DetectionAction::Acquire && decision.candidate_index == 2,
        "refresh did not select highest-score supported candidate");

    candidates = {{tracked, 0.8f}};
    require(select_detection(candidates, image, &tracked, 0.1, 0.8f).action ==
        DetectionAction::KeepTracker, "acquisition threshold must remain strictly greater");
    candidates = {{tracked, 0.4f}};
    require(select_detection(candidates, image, &tracked, 0.1, 0.8f).action ==
        DetectionAction::KeepTracker, "candidate-threshold boundary lost valid support");

    candidates.clear();
    decision = select_detection(candidates, image, &tracked, 0.1, 0.8f);
    require(decision.action == DetectionAction::Invalid && decision.candidate_index == -1,
        "empty detections retained tracker validity");

    candidates = {{tracked, 0.99f}};
    decision = select_detection(candidates, image, &image, 0.1, 0.8f);
    require(decision.action == DetectionAction::Invalid && decision.candidate_index == -1,
        "small candidate authorized giant background tracker");

    candidates = {{{90, 90, 60, 60}, 0.9f}};
    decision = select_detection(candidates, image, &tracked, 0.1, 0.8f);
    require(decision.action == DetectionAction::Acquire && decision.support_iou > 0.4,
        "ordinary target size change lost support");

    const cv::Rect edge_tracker(0, 0, 10, 10);
    candidates = {{{0, 0, 100, 10}, 0.9f}};
    decision = select_detection(candidates, image, &edge_tracker, 0.1, 0.8f);
    require(decision.action == DetectionAction::Acquire && decision.support_iou == 0.1,
        "IoU equality must support refresh");
    require(select_detection(candidates, image, &edge_tracker,
        std::nextafter(0.1, 1.0), 0.8f).action == DetectionAction::Invalid,
        "IoU below requested threshold authorized refresh");
    candidates = {{{-10, -10, 20, 20}, 0.9f}};
    decision = select_detection(candidates, image, &edge_tracker, 1.0, 0.8f);
    require(decision.action == DetectionAction::Acquire && decision.support_iou == 1.0 &&
        decision.box == edge_tracker, "image-edge clipping changed visible support");

    // The same unrelated candidate must be rejected on a validation frame, but
    // may globally reacquire once the caller has cleared tracking on the next frame.
    candidates = {{{900, 500, 40, 40}, 0.99f}};
    require(select_detection(candidates, image, &tracked, 0.1, 0.8f).action ==
        DetectionAction::Invalid, "unsupported refresh became global acquisition");
    decision = select_detection(candidates, image, nullptr, 0.1, 0.8f);
    require(decision.action == DetectionAction::Acquire && decision.candidate_index == 0,
        "global acquisition could not recover after invalidation");
    const cv::Rect invalid(-1, -1, -1, -1);
    require(select_detection(candidates, image, &invalid, 0.1, 0.8f).action ==
        DetectionAction::Invalid, "invalid tracker box became same-frame acquisition");

    candidates = {{{100, 100, 2, 2}, 0.99f}, {tracked, 0.9f}};
    decision = select_detection(candidates, image, nullptr, 0.1, 0.8f);
    require(decision.action == DetectionAction::Invalid && decision.candidate_index == 0,
        "global acquisition incorrectly fell through invalid best box to runner-up");
    candidates = {{tracked, 0.8f}};
    require(select_detection(candidates, image, nullptr, 0.1, 0.8f).action ==
        DetectionAction::Invalid, "global acquisition accepted threshold equality");
    candidates.clear();
    require(select_detection(candidates, image, nullptr, 0.1, 0.8f).action ==
        DetectionAction::Invalid, "empty global acquisition became valid");

    std::cout << "Tracking validation: spatial support, strict acquisition, loss and reacquisition passed\n";
}
