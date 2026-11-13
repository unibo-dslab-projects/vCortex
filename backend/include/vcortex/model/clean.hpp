#pragma once

#include <vector>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

// pcl::StatisticalOutlierRemoval; pcl tutorial values (50, 1.0) cut too much on sparse 5 cm
// voxel maps
std::vector<LidarPoint> drop_outliers(const std::vector<LidarPoint>& in, int mean_k, double stddev_mul);

}  // namespace vcortex
