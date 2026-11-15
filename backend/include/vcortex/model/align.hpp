#pragma once

#include <vector>

#include <Eigen/Core>

namespace vcortex {

/// Returns the turn of the map, radians in [0, pi/2), that makes most walls run along x or y;
/// turning the points by minus it squares the map.
// one entry per occupied cell, not per point, so dense furniture does not outvote walls
double dominant_yaw(const std::vector<Eigen::Vector2d>& band_cells);

}  // namespace vcortex
