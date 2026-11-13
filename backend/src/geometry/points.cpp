#include "vcortex/geometry/points.hpp"

#include <stdexcept>

namespace vcortex {

Eigen::AlignedBox3f point_bounds(const std::vector<LidarPoint>& points) {
    if (points.empty()) throw std::invalid_argument("point_bounds: no points");

    Eigen::AlignedBox3f bounds;
    for (const LidarPoint& point : points) bounds.extend(point.position);
    return bounds;
}

}  // namespace vcortex
