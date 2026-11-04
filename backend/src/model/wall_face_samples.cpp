#include "vcortex/model/wall_face_samples.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace vcortex {

namespace {

int cells_in(double length, double cell) {
    return std::max(1, static_cast<int>(std::lround(length / cell)));
}

// upper middle for an even count: off by one point at most, well under the lidar noise
double median(std::vector<double> v) {
    const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
    std::nth_element(v.begin(), mid, v.end());
    return *mid;
}

}  // namespace

std::vector<WallFaceSample> sample_wall_faces(const RoomSpace& space, const Grid& grid,
                                        const std::vector<Point>& aligned, const Plane& floor,
                                        double low_lo, double band_hi, Direction d,
                                        const OutlineParams& p) {
    std::vector<WallFaceSample> out;
    if (grid.width <= 0 || grid.height <= 0 || space.inside.size() != grid.column.size()) {
        return out;
    }

    // across: the walk's axis; along: the wall's axis, cut into strips
    const bool walk_x = d.axis == Axis::X;
    const int across_n = walk_x ? grid.width : grid.height;
    const int along_n = walk_x ? grid.height : grid.width;
    const double across0 = walk_x ? grid.origin.x : grid.origin.y;
    const double along0 = walk_x ? grid.origin.y : grid.origin.x;
    const int per_strip = cells_in(p.strip, grid.cell);
    const int strips = along_n / per_strip;
    const int max_gap = cells_in(p.max_gap, grid.cell);
    const int max_reach = cells_in(p.max_reach, grid.cell);
    const auto slot = [&](int s, int i) {
        return static_cast<std::size_t>(s) * static_cast<std::size_t>(across_n) +
               static_cast<std::size_t>(i);
    };

    // band plus the coin's low layer: wall bases seen only under desks still count
    std::vector<std::uint8_t> occupied(slot(strips, 0), 0);
    std::vector<std::uint8_t> room(slot(strips, 0), 0);
    std::vector<std::vector<double>> coords(static_cast<std::size_t>(strips));
    for (const Point& pt : aligned) {
        const double h = height_above(floor, pt);
        if (h < low_lo || h >= band_hi) continue;
        const double across = walk_x ? pt.x : pt.y;
        const double along = walk_x ? pt.y : pt.x;
        const int i = static_cast<int>(std::floor((across - across0) / grid.cell));
        const int j = static_cast<int>(std::floor((along - along0) / grid.cell));
        if (i < 0 || i >= across_n || j < 0 || j / per_strip >= strips) continue;
        occupied[slot(j / per_strip, i)] = 1;
        coords[static_cast<std::size_t>(j / per_strip)].push_back(across);
    }
    for (int j = 0; j < strips * per_strip; ++j) {
        for (int i = 0; i < across_n; ++i) {
            const int ix = walk_x ? i : j, iy = walk_x ? j : i;
            const std::size_t cell = static_cast<std::size_t>(iy) *
                                         static_cast<std::size_t>(grid.width) +
                                     static_cast<std::size_t>(ix);
            if (space.inside[cell] != 0) room[slot(j / per_strip, i)] = 1;
        }
    }

    for (int s = 0; s < strips; ++s) {
        int start = -1;
        for (int i = 0; i < across_n; ++i) {
            if (room[slot(s, i)] == 0) continue;
            if (d.outward > 0 || start < 0) start = i;
        }
        if (start < 0) continue;

        WallFaceSample sample;
        sample.from = along0 + s * per_strip * grid.cell;
        sample.to = sample.from + per_strip * grid.cell;

        // past the grid counts as empty: an open side ends at a gap, not at the reach
        int last = 0, empty = 0;
        bool gap = false;
        for (int step = 1; step <= max_reach; ++step) {
            const int i = start + d.outward * step;
            if (i >= 0 && i < across_n && occupied[slot(s, i)] != 0) {
                last = step;
                empty = 0;
            } else if (++empty >= max_gap) {
                gap = true;
                break;
            }
        }

        if (gap && last > 0) {
            const int i = start + d.outward * last;
            // from the outer side of the last occupied cell, face_depth back towards the room
            const double outer = across0 + (d.outward > 0 ? i + 1 : i) * grid.cell;
            const double lo = d.outward > 0 ? outer - p.face_depth : outer;
            const double hi = d.outward > 0 ? outer : outer + p.face_depth;
            std::vector<double> window;
            for (double c : coords[static_cast<std::size_t>(s)]) {
                if (c >= lo && c < hi) window.push_back(c);
            }
            if (!window.empty()) sample.face = median(std::move(window));
        }
        out.push_back(sample);
    }
    return out;
}

}  // namespace vcortex
