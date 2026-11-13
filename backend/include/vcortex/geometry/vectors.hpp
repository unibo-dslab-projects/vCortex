#pragma once

#include <vector>

#include <Eigen/Core>

namespace vcortex {

enum class Axis { X, Y };  // X: constant x, runs along y

double normal_of(Axis a, double x, double y);
double along_of(Axis a, double x, double y);

bool overlaps(double a0, double a1, double b0, double b1);

bool inside(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& box);

}  // namespace vcortex
