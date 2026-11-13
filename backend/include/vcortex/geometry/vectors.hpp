#pragma once

#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

enum class Axis { X, Y };  // X: constant x, runs along y

double normal_of(Axis a, double x, double y);
double along_of(Axis a, double x, double y);

bool overlaps(double a0, double a1, double b0, double b1);

// turn about z by the angle whose cos and sin are given; z stays, so heights need no fix
struct Turn {
    double c = 1.0, s = 0.0;
    Vec2 operator()(double x, double y) const { return {c * x - s * y, s * x + c * y}; }
    Vec2 operator()(const Vec2& v) const { return (*this)(v.x, v.y); }
};

bool inside(const Vec2& p, const std::vector<Vec2>& box);

struct FootprintBox {
    double min_x = 0.0, max_x = 0.0;
    double min_y = 0.0, max_y = 0.0;
};

FootprintBox footprint_box(const std::vector<Vec2>& footprint);

}  // namespace vcortex
