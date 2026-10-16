#pragma once

#include <opencv2/core/types.hpp>

#include <cstddef>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace vision_tracking {

enum class DetectionAction { Invalid, KeepTracker, Acquire };

struct DetectionDecision {
    DetectionAction action = DetectionAction::Invalid;
    int candidate_index = -1;
    cv::Rect box{-1, -1, -1, -1};
    double support_iou = 0.0;
    bool local_recovery = false;
};

inline double intersection_over_union(const cv::Rect& first, const cv::Rect& second)
{
    if (first.width <= 0 || first.height <= 0 || second.width <= 0 || second.height <= 0) {
        return 0.0;
    }
    const auto intersection = first & second;
    const double intersection_area = static_cast<double>(intersection.width) * intersection.height;
    const double union_area = static_cast<double>(first.width) * first.height +
        static_cast<double>(second.width) * second.height - intersection_area;
    return intersection_area / union_area;
}

// Candidates are the unmodified post-NMS array, already subject to the detector's
// candidate threshold. A non-null tracker_box requests support validation, even
// when that box is invalid: loss must not become same-frame global acquisition.
// The template keeps this policy independent of inference sessions for testing.
// IoU is image-clipped; this is spatial support, not a target identity guarantee.
template<typename Candidates>
DetectionDecision select_detection(const Candidates& candidates, const cv::Rect& image_bounds,
    const cv::Rect* tracker_box, double validation_iou, float acquisition_threshold)
{
    DetectionDecision decision;
    if (tracker_box && (tracker_box->width <= 0 || tracker_box->height <= 0)) {
        return decision;
    }
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto& candidate = candidates[index];
        const cv::Rect safe_box = candidate.box & image_bounds;
        const double overlap = tracker_box ? intersection_over_union(*tracker_box, safe_box) : 0.0;
        if (tracker_box && overlap < validation_iou) {
            continue;
        }
        if (decision.candidate_index < 0 ||
            candidate.conf > candidates[decision.candidate_index].conf) {
            decision.candidate_index = static_cast<int>(index);
            decision.box = safe_box;
            decision.support_iou = overlap;
        }
    }
    if (decision.candidate_index < 0) {
        return decision;
    }
    const auto& selected = candidates[decision.candidate_index];
    if (selected.conf > acquisition_threshold) {
        // Preserve global acquisition's top-candidate-only geometry rejection.
        if (decision.box.width > 0 && decision.box.height > 0 &&
            static_cast<double>(decision.box.width) * decision.box.height >= 10.0) {
            decision.action = DetectionAction::Acquire;
        }
    } else if (tracker_box) {
        decision.action = DetectionAction::KeepTracker;
    }
    return decision;
}

// Only detector evidence can renew this budget. Injecting steady-clock instants
// keeps the policy testable without a ROS clock, sleeps, or inference sessions.
class ContinuityBudget {
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    enum class Expiry { None, RecoveryTimeout, NonMonotonicClock };

    ContinuityBudget(double hold_seconds, double recovery_seconds)
        : hold_(hold_seconds), recovery_(recovery_seconds)
    {
        if (!std::isfinite(hold_seconds) || !std::isfinite(recovery_seconds) ||
            hold_seconds <= 0.0 || hold_seconds >= recovery_seconds) {
            throw std::invalid_argument(
                "tracking deadlines must be finite and 0 < validation_hold < recovery_timeout");
        }
    }

    void reset()
    {
        active_ = false;
        probation_ = false;
        frame_gap_ = Clock::duration::zero();
    }

    void acquire(Time now)
    {
        active_ = true;
        probation_ = false;
        last_supported_ = last_frame_ = now;
        frame_gap_ = Clock::duration::zero();
    }

    Expiry expire(Time now)
    {
        if (!active_) return Expiry::None;
        const auto reason = now < last_frame_ || now < last_supported_ ?
            Expiry::NonMonotonicClock :
            (now - last_supported_ > recovery_ ? Expiry::RecoveryTimeout : Expiry::None);
        if (reason != Expiry::None) reset();
        return reason;
    }

    Expiry begin_frame(Time now)
    {
        const auto reason = expire(now);
        if (active_) {
            frame_gap_ = now - last_frame_;
            last_frame_ = now;
        }
        return reason;
    }

    bool support(Time now)
    {
        if (expire(now) != Expiry::None || !active_) return false;
        last_supported_ = now;
        probation_ = false;
        return true;
    }

    void miss()
    {
        if (active_) probation_ = true;
    }

    bool validation_due(Time now) const
    {
        // Predict the next frame's age from the observed callback cadence.
        // At low FPS this validates now rather than extending tracker-only
        // permission until an arbitrary number of frames has elapsed.
        return active_ && (probation_ || now < last_frame_ ||
            now - last_supported_ + frame_gap_ >= hold_);
    }

    bool publication_allowed(Time now) const
    {
        return active_ && now >= last_frame_ && now >= last_supported_ &&
            now - last_supported_ <= hold_;
    }

    bool recovery_allowed(Time now) const
    {
        return active_ && now >= last_frame_ && now >= last_supported_ &&
            now - last_supported_ <= recovery_;
    }

    bool active() const { return active_; }
    bool in_probation() const { return probation_; }
    double support_age(Time now) const
    {
        return std::chrono::duration<double>(now - last_supported_).count();
    }

private:
    std::chrono::duration<double> hold_;
    std::chrono::duration<double> recovery_;
    bool active_ = false;
    bool probation_ = false;
    Time last_supported_{};
    Time last_frame_{};
    Clock::duration frame_gap_{};
};

// Prefer evidence for the current prediction. Only a live probation budget
// permits the last confirmed YOLO box to act as a bounded spatial prior.
template<typename Candidates>
DetectionDecision select_tracking_detection(const Candidates& candidates,
    const cv::Rect& image_bounds, const cv::Rect& tracker_box,
    const cv::Rect& confirmed_anchor, const ContinuityBudget& budget,
    ContinuityBudget::Time now, double validation_iou, float acquisition_threshold)
{
    if (!budget.recovery_allowed(now) || tracker_box.width <= 0 || tracker_box.height <= 0)
        return {};
    auto decision = select_detection(candidates, image_bounds, &tracker_box,
        validation_iou, acquisition_threshold);
    if (decision.action != DetectionAction::Invalid || !budget.in_probation()) {
        return decision;
    }
    decision = select_detection(candidates, image_bounds, &confirmed_anchor,
        validation_iou, acquisition_threshold);
    if (decision.action != DetectionAction::Invalid) {
        decision.action = DetectionAction::Acquire;
        decision.local_recovery = true;
    }
    return decision;
}

}  // namespace vision_tracking
