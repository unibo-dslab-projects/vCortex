#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/model/floor/floor.hpp"

namespace vcortex {

/// Return a vector of objects found in the given points.
std::vector<Object> find_objects(const std::vector<LidarPoint>& furniture_points, const Plane& floor,
                                 const ObjectParams& parameters);

}  // namespace vcortex
