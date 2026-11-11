#pragma once

#include "vcortex/domain/point.hpp"

#include <vector>

namespace vcortex {

// extents of a point set in x, y and z; one per axis, in metres
struct Bounds {
    float min_x = 0.0F, max_x = 0.0F;
    float min_y = 0.0F, max_y = 0.0F;
    float min_z = 0.0F, max_z = 0.0F;
};

// one pass; throws if the point set is empty
Bounds point_bounds(const std::vector<Point>& points);

}  // namespace vcortex
