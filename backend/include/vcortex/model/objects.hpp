#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/point.hpp"
#include "vcortex/domain/settings.hpp"

namespace vcortex {
    std::vector<Object> find_objects(const std::vector<Point>& furniture_points, const std::vector<Wall>& walls, const ObjectParams& params);
} // namespace vcortex
