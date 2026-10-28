#pragma once

#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// Heights are measured from a fitted plane, not from z = 0: the lidar map frame starts where the
// robot stood and is slightly tilted (0.86 degrees on a real map, 11 cm over a 7.6 m room).

// z = a x + b y + c
struct Plane {
    double a, b, c;
};

// Throws if no floor is found.
Plane find_floor(const std::vector<Point>& in);

inline double height_above(const Plane& floor, const Point& p) {
    return p.z - (floor.a * p.x + floor.b * p.y + floor.c);
}

}  // namespace vcortex
