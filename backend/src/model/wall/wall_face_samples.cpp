#include "vcortex/model/wall/wall_face_samples.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "vcortex/geometry/grids.hpp"
#include "vcortex/model/median.hpp"

namespace vcortex {

std::vector<WallFaceSample> sample_wall_faces(const RoomSpace& space, const Grid& grid,
                                        const std::vector<Point>& aligned, const Plane& floor,
                                        double low_lo, double band_hi, Direction d,
                                        const OutlineParams& p) {
    std::vector<WallFaceSample> out;
    if (grid.width <= 0 || grid.height <= 0 || space.inside.size() != grid.column.size()) {
        return out;
    }

    // across: the walk's axis; along: the wall's axis, cut into strips
    const bool search_axis_is_x = d.axis == Axis::X;
    const int across_len = search_axis_is_x ? grid.width : grid.height;
    const int along_len = search_axis_is_x ? grid.height : grid.width;
    const double across_start = search_axis_is_x ? grid.origin.x : grid.origin.y; // grid border starting coordinates
    const double along_start = search_axis_is_x ? grid.origin.y : grid.origin.x;  // grid base starting coordinates
    const int cells_per_strip = cells_in(p.strip, grid.cell);
    const int num_strips = along_len / cells_per_strip;
    const int max_gap = cells_in(p.max_gap, grid.cell);
    const int max_reach = cells_in(p.max_reach, grid.cell);
    const auto get_index = [&](int y, int x) {
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(across_len) +
               static_cast<std::size_t>(x);
    };

    std::vector<std::uint8_t> occupied(get_index(num_strips, 0), 0);
    std::vector<std::uint8_t> room(get_index(num_strips, 0), 0);
    // vector to keep track of occupied cells per strip -> calculate walls by median position
    std::vector<std::vector<double>> coords(static_cast<std::size_t>(num_strips));
    for (const Point& pt : aligned) {
        const double h = height_above(floor, pt);
        if (h < low_lo || h >= band_hi) continue;
        const double across = search_axis_is_x ? pt.x : pt.y;
        const double along = search_axis_is_x ? pt.y : pt.x;
        const int i = static_cast<int>(std::floor((across - across_start) / grid.cell));
        const int j = static_cast<int>(std::floor((along - along_start) / grid.cell));
        if (i < 0 || i >= across_len || j < 0 || j / cells_per_strip >= num_strips) continue;
        occupied[get_index(j / cells_per_strip, i)] = 1;
        coords[static_cast<std::size_t>(j / cells_per_strip)].push_back(across);
    }
    for (int j = 0; j < num_strips * cells_per_strip; ++j) {
        for (int i = 0; i < across_len; ++i) {
            const int ix = search_axis_is_x ? i : j, iy = search_axis_is_x ? j : i;
            const std::size_t cell = static_cast<std::size_t>(iy) *
                                         static_cast<std::size_t>(grid.width) +
                                     static_cast<std::size_t>(ix);
            if (space.inside[cell] != 0) room[get_index(j / cells_per_strip, i)] = 1;
        }
    }

    for (int s = 0; s < num_strips; ++s) {
        int start = -1;
        for (int i = 0; i < across_len; ++i) {
            if (room[get_index(s, i)] == 0) continue;
            if (d.outward > 0 || start < 0) start = i;
        }
        if (start < 0) continue;

        WallFaceSample sample;
        sample.from = along_start + s * cells_per_strip * grid.cell;
        sample.to = sample.from + cells_per_strip * grid.cell;

        // looking for the wall, resetting the counter while finding occupied cells
        // freeing when there is a gap large enough to indicate that the wall ended
        int last = 0, empty = 0;
        bool gap = false;
        for (int step = 1; step <= max_reach; ++step) {
            const int i = start + d.outward * step;
            if (i >= 0 && i < across_len && occupied[get_index(s, i)] != 0) {
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
            const double outer = across_start + (d.outward > 0 ? i + 1 : i) * grid.cell;
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
