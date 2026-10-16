#include "tracking_validation.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>
#include <utility>
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

void test_continuity_budget()
{
    using vision_tracking::ContinuityBudget;
    using vision_tracking::DetectionAction;
    using vision_tracking::select_tracking_detection;
    using Time = ContinuityBudget::Time;
    using Expiry = ContinuityBudget::Expiry;
    const auto at = [](int milliseconds) {
        return Time{} + std::chrono::milliseconds(milliseconds);
    };
    const cv::Rect image(0, 0, 1920, 1080);
    const cv::Rect tracked(100, 100, 40, 40);
    const cv::Rect drifted(900, 500, 40, 40);
    std::vector<Candidate> candidates;
    ContinuityBudget budget(0.35, 2.0);

    require(!budget.active() && !budget.publication_allowed(at(0)),
        "unacquired budget authorized output");
    budget.acquire(at(0));
    require(budget.begin_frame(at(100)) == Expiry::None &&
        !budget.validation_due(at(100)), "ordinary tracker frame validated too early");
    require(budget.begin_frame(at(180)) == Expiry::None &&
        !budget.validation_due(at(180)), "short callback cadence consumed support early");
    require(budget.begin_frame(at(270)) == Expiry::None &&
        budget.validation_due(at(270)), "next low-FPS frame crossing hold was not anticipated");
    auto decision = select_tracking_detection(candidates, image, tracked, tracked,
        budget, at(270), 0.1, 0.8f);
    require(decision.action == DetectionAction::Invalid, "empty validation was support");
    budget.miss();
    require(budget.active() && budget.in_probation() &&
        budget.publication_allowed(at(270)),
        "single missing validation immediately destroyed tracker or publication");
    require(budget.begin_frame(at(300)) == Expiry::None &&
        budget.validation_due(at(300)), "probation did not validate every frame");
    candidates = {{tracked, 0.452f}};
    decision = select_tracking_detection(candidates, image, tracked, tracked,
        budget, at(300), 0.1, 0.8f);
    require(decision.action == DetectionAction::KeepTracker && !decision.local_recovery,
        "short missing interval required global high-score acquisition");
    require(budget.support(at(300)) && !budget.in_probation() &&
        budget.publication_allowed(at(300)), "low-score support failed to recover budget");
    budget.begin_frame(at(330));
    require(!budget.validation_due(at(330)), "recovery failed to restore normal refresh cadence");

    budget.acquire(at(0));
    budget.begin_frame(at(200));
    budget.miss();
    require(budget.publication_allowed(at(350)),
        "hold equality revoked permission before the configured boundary");
    require(budget.begin_frame(at(351)) == Expiry::None &&
        !budget.publication_allowed(at(351)) && budget.active(),
        "hold expiration destroyed the recoverable internal template");
    decision = select_tracking_detection(candidates, image, tracked, tracked,
        budget, at(351), 0.1, 0.8f);
    require(decision.action == DetectionAction::KeepTracker &&
        budget.support(at(351)) && budget.publication_allowed(at(351)),
        "held-invalid output could not regain permission from low-score spatial support");

    // The current prediction has priority, even over a high-score candidate
    // matching an old anchor. An anchor-only candidate may rebuild a drifted
    // template, but only during bounded probation.
    budget.acquire(at(0));
    candidates = {{tracked, 0.99f}, {drifted, 0.6f}};
    budget.begin_frame(at(100));
    budget.miss();
    decision = select_tracking_detection(candidates, image, drifted, tracked,
        budget, at(100), 0.1, 0.8f);
    require(decision.action == DetectionAction::KeepTracker &&
        decision.candidate_index == 1 && !decision.local_recovery,
        "old anchor stole a currently supported prediction");
    candidates = {{tracked, 0.452f}};
    decision = select_tracking_detection(candidates, image, drifted, tracked,
        budget, at(100), 0.1, 0.8f);
    require(decision.action == DetectionAction::Acquire &&
        decision.candidate_index == 0 && decision.local_recovery,
        "probation anchor could not rebuild the same target from low-score evidence");
    require(select_tracking_detection(candidates, image, cv::Rect(-1, -1, -1, -1),
        tracked, budget, at(100), 0.1, 0.8f).action == DetectionAction::Invalid,
        "invalid geometry revived via the probation anchor");
    budget.support(at(100));
    decision = select_tracking_detection(candidates, image, drifted, tracked,
        budget, at(100), 0.1, 0.8f);
    require(decision.action == DetectionAction::Invalid,
        "anchor recovery was permitted outside probation");

    budget.acquire(at(0));
    budget.miss();
    require(budget.begin_frame(at(2000)) == Expiry::None && budget.active() &&
        !budget.publication_allowed(at(2000)), "recovery equality expired prematurely");
    decision = select_tracking_detection(candidates, image, drifted, tracked,
        budget, at(2000), 0.1, 0.8f);
    require(decision.local_recovery, "bounded recovery rejected exact timeout equality");
    // Selection also checks the deadline itself: the caller cannot accidentally
    // use an old anchor before noticing a dropped-frame timeout.
    decision = select_tracking_detection(candidates, image, drifted, tracked,
        budget, at(2001), 0.1, 0.8f);
    require(decision.action == DetectionAction::Invalid,
        "stale low-score prior revived on its expiration frame");
    require(budget.begin_frame(at(2001)) == Expiry::RecoveryTimeout &&
        !budget.active() && !budget.in_probation() && !budget.support(at(2001)),
        "long no-support interval retained or revived the template");
    require(vision_tracking::select_detection(candidates, image, nullptr, 0.1, 0.8f).action ==
        DetectionAction::Invalid, "expired target bypassed strict global acquisition");
    candidates[0].conf = std::nextafter(0.8f, 1.0f);
    require(vision_tracking::select_detection(candidates, image, nullptr, 0.1, 0.8f).action ==
        DetectionAction::Acquire, "global recovery rejected a score strictly above threshold");

    // Tracker-only frames and their own confidence/positive geometry never
    // renew detector permission, even with irregular delivery.
    budget.acquire(at(0));
    for (const int frame_ms : {40, 90, 200, 349, 351, 999, 1900}) {
        require(budget.begin_frame(at(frame_ms)) == Expiry::None && budget.active(),
            "tracker-only cadence reset the recoverable template early");
        require(budget.publication_allowed(at(frame_ms)) == (frame_ms <= 350),
            "tracker-only frame renewed unsupported output permission");
    }
    require(budget.begin_frame(at(2200)) == Expiry::RecoveryTimeout,
        "dropped frames failed to consume the recovery deadline");
    budget.acquire(at(0));
    budget.begin_frame(at(300));
    budget.miss();
    require(budget.begin_frame(at(299)) == Expiry::NonMonotonicClock &&
        !budget.active() && !budget.support(at(300)),
        "backward injected time extended the previous template lifetime");
    budget.acquire(at(1000));
    budget.reset();
    require(!budget.active() && !budget.in_probation() &&
        !budget.publication_allowed(at(1001)) && !budget.support(at(1001)),
        "geometry/data reset retained continuity authorization");
    budget.acquire(at(0));
    require(budget.expire(at(2001)) == Expiry::RecoveryTimeout &&
        !budget.support(at(2001)), "slow inference renewed already-expired support");

    for (const auto& deadlines : {std::pair<double, double>{0.0, 2.0},
             {0.35, 0.35}, {2.0, 0.35}, {-0.1, 2.0},
             {std::numeric_limits<double>::quiet_NaN(), 2.0},
             {0.35, std::numeric_limits<double>::infinity()}}) {
        bool rejected = false;
        try {
            const ContinuityBudget invalid(deadlines.first, deadlines.second);
            (void)invalid;
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "invalid continuity deadline configuration was accepted");
    }
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

    test_continuity_budget();
    std::cout << "Tracking validation: spatial support, strict acquisition, bounded continuity passed\n";
}
