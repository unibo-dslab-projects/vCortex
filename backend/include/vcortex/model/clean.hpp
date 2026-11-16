#pragma once

#include <vector>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

/// Drops the points whose mean distance to their mean_k nearest neighbours lies more than
/// stddev_mul standard deviations above the mean of all points; keeps the input order.
// pcl::StatisticalOutlierRemoval; pcl tutorial values (50, 1.0) cut too much on sparse 5 cm
// voxel maps
std::vector<LidarPoint> drop_outliers(const std::vector<LidarPoint>& points, int mean_k,
                                      double stddev_mul);

}  // namespace vcortex
