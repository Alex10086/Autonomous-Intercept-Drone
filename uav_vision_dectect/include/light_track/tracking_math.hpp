#pragma once

#include <limits>

namespace lighttrack {

// Nonempty model tensors share one flat row-major candidate index. Never turn
// the winning index into floating row/column coordinates and truncate it back.
inline int select_candidate(const float *classification, const float *penalty,
                            const float *window, int count, float window_influence)
{
    int best_index = 0;
    float best_score = -std::numeric_limits<float>::infinity();
    for (int index = 0; index < count; ++index) {
        const float score = (penalty[index] * classification[index]) *
            (1.0f - window_influence) + window[index] * window_influence;
        if (score > best_score) {
            best_score = score;
            best_index = index;
        }
    }
    return best_index;
}

}  // namespace lighttrack
