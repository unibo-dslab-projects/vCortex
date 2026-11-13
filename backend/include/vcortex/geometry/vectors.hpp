#pragma once

#include <vector>

#include <Eigen/Core>

namespace vcortex {

enum class Axis { X, Y };

/// Axis a is the wall's normal: X for a wall of constant x (it runs along y).
/// Returns the point's coordinate on that axis, i.e. how far across the wall it lies
double normal_of(Axis a, double x, double y);
/// Axis a is the wall's normal: X for a wall of constant x (it runs along y).
/// Returns the point's coordinate on the other axis, i.e. where along the wall it lies
double along_of(Axis a, double x, double y);
/// Check that the two intervals [a0, a1] and [b0, b1] intersect
bool overlaps(double a0, double a1, double b0, double b1);
/// Check that the point is inside the polygon, given as its corners in order
bool inside(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

}  // namespace vcortex
