#pragma once

#include <opencv2/core/types.hpp>

#include <cstddef>

namespace vision_tracking {

enum class DetectionAction { Invalid, KeepTracker, Acquire };

struct DetectionDecision {
    DetectionAction action = DetectionAction::Invalid;
    int candidate_index = -1;
    cv::Rect box{-1, -1, -1, -1};
    double support_iou = 0.0;
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

}  // namespace vision_tracking
