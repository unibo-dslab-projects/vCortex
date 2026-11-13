#include "vcortex/geometry/vectors.hpp"

#include <cstddef>

namespace vcortex {

double normal_of(Axis a, double x, double y) { return a == Axis::X ? x : y; }

double along_of(Axis a, double x, double y) { return a == Axis::X ? y : x; }

bool overlaps(double a0, double a1, double b0, double b1) { return a0 < b1 && b0 < a1; }

bool inside(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {
    // ray casting: a ray from the point to +x crosses the edges an odd number of times
    // only when the point is inside, whatever the polygon's shape
    bool is_inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const Eigen::Vector2d& current = polygon[i];
        const Eigen::Vector2d& previous = polygon[j];
        const bool edge_spans_point_y = (current.y() > point.y()) != (previous.y() > point.y());
        if (!edge_spans_point_y) continue;
        const double crossing_x = current.x() + (point.y() - current.y()) *
                                                    (previous.x() - current.x()) /
                                                    (previous.y() - current.y());
        if (point.x() < crossing_x) is_inside = !is_inside;
    }
    return is_inside;
}

}  // namespace vcortex
