#include "vcortex/geometry/vectors.hpp"

#include <algorithm>

namespace vcortex {

double normal_of(Axis a, double x, double y) { return a == Axis::X ? x : y; }
double along_of(Axis a, double x, double y) { return a == Axis::X ? y : x; }

bool overlaps(double a0, double a1, double b0, double b1) { return a0 < b1 && b0 < a1; }

bool inside(const Vec2& p, const std::vector<Vec2>& box) {
    return p.x >= box[0].x && p.x <= box[2].x && p.y >= box[0].y && p.y <= box[2].y;
}

FootprintBox footprint_box(const std::vector<Vec2>& footprint) {
    FootprintBox box{footprint.front().x, footprint.front().x, footprint.front().y,
                     footprint.front().y};
    for (const Vec2& vertex : footprint) {
        box.min_x = std::min(box.min_x, vertex.x);
        box.max_x = std::max(box.max_x, vertex.x);
        box.min_y = std::min(box.min_y, vertex.y);
        box.max_y = std::max(box.max_y, vertex.y);
    }
    return box;
}

}  // namespace vcortex
