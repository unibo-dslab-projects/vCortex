#pragma once

#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// heights from a fitted plane, not z = 0: the map frame starts where the robot stood and is
// tilted up to ~1 degree on real maps (~11 cm across a 7.6 m room)

// z = a x + b y + c
struct Plane {
    double a = 0.0, b = 0.0, c = 0.0;
};

// throws if no floor is found
Plane find_floor(const std::vector<Point>& in);

inline double height_above(const Plane& floor, const Point& p) {
    return p.z - (floor.a * p.x + floor.b * p.y + floor.c);
}

}  // namespace vcortex
