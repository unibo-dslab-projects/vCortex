#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// Stage A (step 5): remove points that are not part of the building.
// Run drop_transient first: a person's trail would distort the neighbour statistics.

// Keep points with hits >= min_scans.
std::vector<Point> drop_transient(const std::vector<Point>& in, std::uint16_t min_scans);

// Statistical outlier removal (pcl::StatisticalOutlierRemoval).
std::vector<Point> drop_outliers(const std::vector<Point>& in, int mean_k, double stddev_mul);

}  // namespace vcortex
