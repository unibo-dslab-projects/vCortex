#include "vcortex/model/object_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "vcortex/geometry/points.hpp"

namespace vcortex {

namespace {

// cells past the border are skipped: dilate never grows from them, and erode counts them as
// full, so an object at the edge is never eaten
bool has_neighbour_with_value(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                              int row, int column, int radius_cells, std::uint8_t value) {
    for (int row_offset = -radius_cells; row_offset <= radius_cells; ++row_offset) {
        for (int column_offset = -radius_cells; column_offset <= radius_cells; ++column_offset) {
            const int neighbour_row = row + row_offset;
            const int neighbour_column = column + column_offset;
            if (neighbour_row < 0 || neighbour_row >= grid.height || neighbour_column < 0 ||
                neighbour_column >= grid.width) {
                continue;
            }
            const auto index =
                static_cast<std::size_t>(neighbour_row * grid.width + neighbour_column);
            if (mask[index] == value) return true;
        }
    }
    return false;
}

}  // namespace

ObjectGrid build_object_grid(const std::vector<Point>& furniture_points, const Plane& floor,
                             double cell, int padding_cells, int cell_min_points) {
    ObjectGrid grid;
    grid.cell = cell;
    if (furniture_points.empty()) return grid;

    const Bounds bounds = point_bounds(furniture_points);

    // adding padding to support spatial algorithms boundary conditions
    const double padding_metres = padding_cells * cell;
    grid.origin = {bounds.min_x - padding_metres, bounds.min_y - padding_metres};
    grid.width =
        static_cast<int>(std::floor((bounds.max_x - bounds.min_x) / cell)) + 1 + 2 * padding_cells;
    grid.height = static_cast<int>(std::floor((bounds.max_y - bounds.min_y) / cell)) +
                  1 + 2 * padding_cells;

    const auto cell_count =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    grid.is_occupied.assign(cell_count, 0);
    grid.point_count.assign(cell_count, 0);
    grid.top_metres.assign(cell_count, 0.0F);
    grid.cell_of_point.reserve(furniture_points.size());

    // voxeling the points cloud in a 2D grid, counting points per cell
    for (const Point& point : furniture_points) {
        // clamped: rounding can push a point on the max edge one cell past the data
        const int column = std::clamp(static_cast<int>(std::floor((point.x - grid.origin.x) / cell)),
                                      0, grid.width - 1);
        const int row = std::clamp(static_cast<int>(std::floor((point.y - grid.origin.y) / cell)),
                                   0, grid.height - 1);
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

std::vector<std::uint8_t> dilate(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                                 int radius_cells) {
    std::vector<std::uint8_t> result(mask.size(), 0);
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const bool is_full = has_neighbour_with_value(grid, mask, row, column, radius_cells, 1);
            result[static_cast<std::size_t>(row * grid.width + column)] = is_full;
        }
    }
    return result;
}

std::vector<std::uint8_t> erode(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                                int radius_cells) {
    std::vector<std::uint8_t> result(mask.size(), 0);
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const bool is_full = !has_neighbour_with_value(grid, mask, row, column, radius_cells, 0);
            result[static_cast<std::size_t>(row * grid.width + column)] = is_full;
        }
    }
    return result;
}

}  // namespace vcortex
