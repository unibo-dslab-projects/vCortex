#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

// one pass; throws if the point set is empty
Eigen::AlignedBox3f point_bounds(const std::vector<LidarPoint>& points);

}  // namespace vcortex
