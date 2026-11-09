#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace vcortex {

// upper middle for an even count: off by one value at most, well under the lidar noise
// takes the values by copy because nth_element reorders them; the caller must pass at least one
inline double median(std::vector<double> values) {
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

}  // namespace vcortex
