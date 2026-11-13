#pragma once

#include <vector>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

// heights from a fitted plane, not z = 0: the map frame starts where the robot stood and is
// tilted up to ~1 degree on real maps (~11 cm across a 7.6 m room)

// z = a x + b y + c
struct Plane {
    double a = 0.0, b = 0.0, c = 0.0;
};

// throws if no floor is found
Plane find_floor(const std::vector<LidarPoint>& in);

inline double height_above(const Plane& floor, const LidarPoint& p) {
    return p.position.z() - (floor.a * p.position.x() + floor.b * p.position.y() + floor.c);
}

}  // namespace vcortex
