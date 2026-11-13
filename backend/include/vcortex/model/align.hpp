#pragma once

#include <vector>

#include <Eigen/Core>

namespace vcortex {

// radians in [0, pi/2); one entry per occupied cell, not per point, so dense furniture
// does not outvote walls
double dominant_yaw(const std::vector<Eigen::Vector2d>& band_cells);

}  // namespace vcortex
