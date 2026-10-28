#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// Run drop_transient first: a person's trail would distort drop_outliers' neighbour statistics.

// A voxel seen in fewer than min_scans scans is a passer-by or a sensor ghost, not structure.
std::vector<Point> drop_transient(const std::vector<Point>& in, std::uint16_t min_scans);

// pcl::StatisticalOutlierRemoval. PCL's tutorial values (50, 1.0) remove too much on a 5 cm voxel
// map, which is much sparser than the clouds they were chosen for.
std::vector<Point> drop_outliers(const std::vector<Point>& in, int mean_k, double stddev_mul);

}  // namespace vcortex
