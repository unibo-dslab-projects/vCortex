#pragma once

#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// Stage C (step 7): the angle that turns the map so its walls run along x and y.
// Radians, in [0, pi/2). One entry per occupied cell, so dense furniture does not outvote walls.
double dominant_yaw(const std::vector<Vec2>& band_cells);

}  // namespace vcortex
