#pragma once

#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// Stage B (step 6): the floor plane, and heights measured from it.

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
