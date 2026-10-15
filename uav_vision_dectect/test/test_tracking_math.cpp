#include "tracking_math.hpp"
#include <array>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
}

int main()
{
    std::array<float, 324> classification, penalty, window;
    penalty.fill(1.0f);
    window.fill(0.0f);
    // Every model cell must index its own regression tensor, including cells
    // such as 21 that the old float row/column conversion decoded as 20.
    for (int candidate = 0; candidate < 324; ++candidate) {
        classification.fill(0.1f);
        classification[candidate] = 0.9f;
        require(lighttrack::select_candidate(classification.data(), penalty.data(),
            window.data(), 324, 0.45f) == candidate, "winning cell changed before bbox decode");
    }
    classification.fill(0.1f);
    classification[21] = 0.95f;
    classification[171] = 0.5f;
    penalty[21] = 0.1f;
    window[171] = 1.0f;
    require(lighttrack::select_candidate(classification.data(), penalty.data(), window.data(),
        324, 0.45f) == 171, "raw argmax substituted for window/size-penalized candidate");
    penalty.fill(1.0f);
    window.fill(0.0f);
    classification.fill(0.0f);
    classification[17] = classification[18] = 0.9f;
    require(lighttrack::select_candidate(classification.data(), penalty.data(), window.data(),
        324, 0.45f) == 17, "tied candidates must retain first-candidate precedence");
    std::cout << "LightTrack: 324 exact regression indices, weighted selection and boundary ties passed\n";
}
