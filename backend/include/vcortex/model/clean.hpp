#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// run drop_transient first: a person's trail skews drop_outliers' neighbour stats

// voxel seen in fewer than min_scans scans: passer-by or sensor ghost, not structure
std::vector<Point> drop_transient(const std::vector<Point>& in, std::uint16_t min_scans);

// pcl::StatisticalOutlierRemoval; pcl tutorial values (50, 1.0) cut too much on sparse 5 cm
// voxel maps
std::vector<Point> drop_outliers(const std::vector<Point>& in, int mean_k, double stddev_mul);

}  // namespace vcortex
