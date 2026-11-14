#include "vcortex/model/object/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "vcortex/geometry/grids.hpp"
#include "vcortex/geometry/points.hpp"

namespace vcortex {

ObjectGrid build_object_grid(const std::vector<LidarPoint>& furniture_points, const Plane& floor,
                             double cell, int padding_cells, int cell_min_points) {
    ObjectGrid grid;
    grid.cell = cell;
    if (furniture_points.empty()) return grid;

    const Eigen::AlignedBox3f bounds = get_points_bounding_box(furniture_points);

    // adding padding to support spatial algorithms boundary conditions
    const double padding_metres = padding_cells * cell;
    grid.origin = {bounds.min().x() - padding_metres, bounds.min().y() - padding_metres};
    grid.width =
        static_cast<int>(std::floor((bounds.max().x() - bounds.min().x()) / cell)) + 1 + 2 * padding_cells;
    grid.height = static_cast<int>(std::floor((bounds.max().y() - bounds.min().y()) / cell)) +
                  1 + 2 * padding_cells;

    const auto cell_count =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    grid.is_occupied.assign(cell_count, 0);
    grid.point_count.assign(cell_count, 0);
    grid.top_metres.assign(cell_count, 0.0F);
    grid.cell_of_point.reserve(furniture_points.size());

    // voxeling the points cloud in a 2D grid, counting points per cell
    for (const LidarPoint& point : furniture_points) {
        // clamped: rounding can push a point on the max edge one cell past the data
        const int column = std::clamp(
            static_cast<int>(std::floor((point.position.x() - grid.origin.x()) / cell)), 0,
            grid.width - 1);
        const int row = std::clamp(
            static_cast<int>(std::floor((point.position.y() - grid.origin.y()) / cell)), 0,
            grid.height - 1);
        const std::size_t index = static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
                                  static_cast<std::size_t>(column);
        ++grid.point_count[index];
        grid.top_metres[index] =
            std::max(grid.top_metres[index], static_cast<float>(height_above(floor, point)));
        grid.cell_of_point.push_back(index);
    }

    for (std::size_t index = 0; index < cell_count; ++index) {
        grid.is_occupied[index] =
            grid.point_count[index] >= static_cast<std::uint32_t>(cell_min_points) ? 1 : 0;
    }
    return grid;
}

std::vector<std::uint8_t> grow_by_cell(const ObjectGrid& grid,
                                       const std::vector<std::uint8_t>& marked_cells,
                                       int radius_cells) {
    std::vector<std::uint8_t> grown(marked_cells.size(), 0);
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const bool is_full =
                has_neighbour_with_value(grid, marked_cells, row, column, radius_cells, 1);
            grown[static_cast<std::size_t>(row * grid.width + column)] = is_full;
        }
    }
    return grown;
}

std::vector<std::uint8_t> shrink_by_cell(const ObjectGrid& grid,
                                         const std::vector<std::uint8_t>& marked_cells,
                                         int radius_cells) {
    std::vector<std::uint8_t> shrunk(marked_cells.size(), 0);
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const bool is_square_full =
                !has_neighbour_with_value(grid, marked_cells, row, column, radius_cells, 0);
            shrunk[static_cast<std::size_t>(row * grid.width + column)] = is_square_full;
        }
    }
    return shrunk;
}

}  // namespace vcortex
