#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/model/floor/floor.hpp"

namespace vcortex {

// aligned frame in and out; walls must already carry their ids, objects point at them
std::vector<Object> find_objects(const std::vector<Point>& furniture_points, const Plane& floor,
                                 const std::vector<Wall>& walls, const ObjectParams& parameters);

}  // namespace vcortex
