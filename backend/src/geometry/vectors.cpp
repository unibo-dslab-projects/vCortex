#include "vcortex/geometry/vectors.hpp"

namespace vcortex {

double normal_of(Axis a, double x, double y) { return a == Axis::X ? x : y; }
double along_of(Axis a, double x, double y) { return a == Axis::X ? y : x; }

bool overlaps(double a0, double a1, double b0, double b1) { return a0 < b1 && b0 < a1; }

bool inside(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& box) {
    return p.x() >= box[0].x() && p.x() <= box[2].x() && p.y() >= box[0].y() && p.y() <= box[2].y();
}

}  // namespace vcortex
