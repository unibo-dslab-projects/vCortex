#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

/// Returns the bounding box of the given points
Eigen::AlignedBox3f get_points_bounding_box(const std::vector<LidarPoint>& points);

}  // namespace vcortex
