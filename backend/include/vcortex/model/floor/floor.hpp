#pragma once

#include <vector>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

// z = a x + b y + c
struct Plane {
    double a = 0.0, b = 0.0, c = 0.0;
};

/// Finds the floor plane passing through the given points.
/// Throws if no floor is found.
Plane find_floor(const std::vector<LidarPoint>& in);

/// Returns the height above the floor for a given point.
inline double height_above(const Plane& floor, const LidarPoint& p) {
    return p.position.z() - (floor.a * p.position.x() + floor.b * p.position.y() + floor.c);
}

}  // namespace vcortex
