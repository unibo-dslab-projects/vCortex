#pragma once

#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// radians in [0, pi/2); one entry per occupied cell, not per point, so dense furniture
// does not outvote walls
double dominant_yaw(const std::vector<Vec2>& band_cells);

}  // namespace vcortex
