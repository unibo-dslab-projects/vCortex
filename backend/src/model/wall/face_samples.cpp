#include "vcortex/model/wall/face_samples.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "vcortex/geometry/grids.hpp"
#include "vcortex/model/median.hpp"

namespace vcortex {

std::vector<WallFaceSample> sample_wall_faces(const FreeSpace& space, const Grid& grid,
                                              const std::vector<LidarPoint>& aligned_points,
                                              const Plane& floor, double band_bottom_metres,
                                              double band_top_metres, Direction direction,
                                              const OutlineParams& parameters) {
    std::vector<WallFaceSample> out;
    if (grid.width <= 0 || grid.height <= 0 || space.room_of_cell.size() != grid.column.size()) {
        return out;
    }

    // across: the walk's axis; along: the wall's axis, cut into strips
    const bool search_axis_is_x = direction.axis == Axis::X;
    const int across_len = search_axis_is_x ? grid.width : grid.height;
    const int along_len = search_axis_is_x ? grid.height : grid.width;
    const double across_start = search_axis_is_x ? grid.origin.x() : grid.origin.y();  // metres
    const double along_start = search_axis_is_x ? grid.origin.y() : grid.origin.x();   // metres
    const int cells_per_strip = num_cells_in_len(parameters.strip, grid.cell);
    const int num_strips = along_len / cells_per_strip;
    const int max_gap = num_cells_in_len(parameters.max_gap, grid.cell);
    const int max_reach = num_cells_in_len(parameters.max_reach, grid.cell);
    // each strip is flattened into one row: index = strip * across_len + across cell
    const auto get_index = [&](int y, int x) {
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(across_len) +
               static_cast<std::size_t>(x);
    };

    std::vector<std::uint8_t> occupied(get_index(num_strips, 0), 0);
    std::vector<std::uint8_t> room(get_index(num_strips, 0), 0);
    // exact across coordinates per strip: cells give only 5 cm steps, the median of these gives
    // the face itself
    std::vector<std::vector<double>> coords(static_cast<std::size_t>(num_strips));
    for (const LidarPoint& pt : aligned_points) {
        const double h = height_above(floor, pt);
        if (h < band_bottom_metres || h >= band_top_metres) continue;
        const double across = search_axis_is_x ? pt.position.x() : pt.position.y();
        const double along = search_axis_is_x ? pt.position.y() : pt.position.x();
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
            if (space.room_of_cell[cell] != 0) room[get_index(j / cells_per_strip, i)] = 1;
        }
    }

    for (int s = 0; s < num_strips; ++s) {
        // start from the outermost free-space cell: furniture in the middle of the room lies
        // before it and is never walked through
        int start = -1;
        for (int i = 0; i < across_len; ++i) {
            if (room[get_index(s, i)] == 0) continue;
            if (direction.outward > 0 || start < 0) start = i;
        }
        if (start < 0) continue;

        WallFaceSample sample;
        sample.from = along_start + s * cells_per_strip * grid.cell;
        sample.to = sample.from + cells_per_strip * grid.cell;

        // keep the last occupied cell before max_gap empty cells: a desk against the wall has
        // shorter gaps under it, so the walk goes through it to the wall behind
        int last = 0, empty = 0;
        bool gap = false;
        for (int step = 1; step <= max_reach; ++step) {
            const int i = start + direction.outward * step;
            if (i >= 0 && i < across_len && occupied[get_index(s, i)] != 0) {
                last = step;
                empty = 0;
            } else if (++empty >= max_gap) {
                gap = true;
                break;
            }
        }

        if (gap && last > 0) {
            const int i = start + direction.outward * last;
            // the lidar sees only the inner face, so the last occupied cell holds it; a window
            // face_depth deep from that cell's outer side keeps furniture in front out
            const double outer = across_start + (direction.outward > 0 ? i + 1 : i) * grid.cell;
            const double lo = direction.outward > 0 ? outer - parameters.face_depth : outer;
            const double hi = direction.outward > 0 ? outer : outer + parameters.face_depth;
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
