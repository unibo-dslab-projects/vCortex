#include "vcortex/geometry/points.hpp"

#include <algorithm>
#include <stdexcept>

namespace vcortex {

Bounds point_bounds(const std::vector<Point>& points) {
    if (points.empty()) throw std::invalid_argument("point_bounds: no points");

    const Point& seed = points.front();
    Bounds b{seed.x, seed.x, seed.y, seed.y, seed.z, seed.z};
    for (const Point& point : points) {
        b.min_x = std::min(b.min_x, point.x);
        b.max_x = std::max(b.max_x, point.x);
        b.min_y = std::min(b.min_y, point.y);
        b.max_y = std::max(b.max_y, point.y);
        b.min_z = std::min(b.min_z, point.z);
        b.max_z = std::max(b.max_z, point.z);
    }
    return b;
}

}  // namespace vcortex
