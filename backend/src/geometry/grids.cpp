#include "vcortex/geometry/grids.hpp"

#include <algorithm>
#include <cmath>

namespace vcortex {

namespace {
    constexpr double kSlack = 1e-9;
} // namespace

bool is_in_grid(const Grid& grid, int x, int y) {
    return x >= 0 && x < grid.width && y >= 0 && y < grid.height;
}

std::size_t at(const Grid& grid, int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(x);
}

bool cell_of(const Grid& g, double x, double y, int& ix, int& iy) {
    ix = static_cast<int>(std::floor((x - g.origin.x()) / g.cell));
    iy = static_cast<int>(std::floor((y - g.origin.y()) / g.cell));
    return is_in_grid(g, ix, iy);
}

std::vector<Eigen::Vector2i> disk(double radius_cells) {
    const int reach = static_cast<int>(std::floor(radius_cells + kSlack));
    std::vector<Eigen::Vector2i> out;

    // keeping only the points within the circle
    for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
            if (dx * dx + dy * dy <= radius_cells * radius_cells + kSlack) out.push_back({dx, dy});
        }
    }

    return out;
}

std::vector<std::uint8_t> grow(const std::vector<std::uint8_t>& marked_cells, const Grid& grid,
                               const std::vector<Eigen::Vector2i>& offsets) {
    std::vector<std::uint8_t> grown_cells(marked_cells.size(), 0);
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (marked_cells[at(grid, x, y)] == 0) continue;
            for (const Eigen::Vector2i& offset : offsets) {
                const int neighbour_x = x + offset.x(), neighbour_y = y + offset.y();
                if (is_in_grid(grid, neighbour_x, neighbour_y)) {
                    grown_cells[at(grid, neighbour_x, neighbour_y)] = 1;
                }
            }
        }
    }
    return grown_cells;
}

std::vector<std::uint8_t> invert(std::vector<std::uint8_t> marked_cells) {
    for (std::uint8_t& cell : marked_cells) cell = static_cast<std::uint8_t>(cell == 0);
    return marked_cells;
}

std::vector<std::uint8_t> close_holes(const std::vector<std::uint8_t>& marked_cells,
                                      const Grid& grid) {
    const std::vector<Eigen::Vector2i> one_cell_disk = disk(1.0);
    return invert(grow(invert(grow(marked_cells, grid, one_cell_disk)), grid, one_cell_disk));
}

int cell_index(double v, double origin, double cell, int size) {
    // clamp: float rounding can put the max point one cell past the end
    return std::clamp(static_cast<int>(std::floor((v - origin) / cell)), 0, size - 1);
}

int num_cells_in_len(double length, double cell) {
    return std::max(1, static_cast<int>(std::lround(length / cell)));
}

bool has_neighbour_with_value(const ObjectGrid& grid, const std::vector<std::uint8_t>& marked_cells,
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
            if (marked_cells[index] == value) return true;
        }
    }
    return false;
}

std::vector<std::size_t> get_neighbour_indices(const ObjectGrid& grid, std::size_t index) {
    const int row = static_cast<int>(index) / grid.width;
    const int column = static_cast<int>(index) % grid.width;
    std::vector<std::size_t> neighbours;
    const int steps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& step : steps) {
        const int neighbour_row = row + step[0];
        const int neighbour_column = column + step[1];
        if (neighbour_row < 0 || neighbour_row >= grid.height || neighbour_column < 0 ||
            neighbour_column >= grid.width) {
            continue;
        }
        neighbours.push_back(static_cast<std::size_t>(neighbour_row * grid.width + neighbour_column));
    }
    return neighbours;
}

long long doubled_signed_area(const std::vector<Eigen::Vector2i>& polygon) {
    long long doubled_area = 0;
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const Eigen::Vector2i& current = polygon[index];
        const Eigen::Vector2i& next = polygon[(index + 1) % polygon.size()];
        doubled_area += static_cast<long long>(current.x()) * next.y() -
                        static_cast<long long>(next.x()) * current.y();
    }
    return doubled_area;
}

}  // namespace vcortex
