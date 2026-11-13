#include "vcortex/geometry/grids.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace vcortex {

namespace {
    constexpr double kSlack = 1e-9;
} // namespace

bool in_grid(const Grid& grid, int x, int y) {
    return x >= 0 && x < grid.width && y >= 0 && y < grid.height;
}

std::size_t at(const Grid& grid, int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(x);
}

bool cell_of(const Grid& g, double x, double y, int& ix, int& iy) {
    ix = static_cast<int>(std::floor((x - g.origin.x) / g.cell));
    iy = static_cast<int>(std::floor((y - g.origin.y) / g.cell));
    return in_grid(g, ix, iy);
}

std::vector<Offset> disk(double radius_cells) {
    const int reach = static_cast<int>(std::floor(radius_cells + kSlack));
    std::vector<Offset> out;

    // keeping only the points within the circle
    for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
            if (dx * dx + dy * dy <= radius_cells * radius_cells + kSlack) out.push_back({dx, dy});
        }
    }

    return out;
}

Mask grow(const Mask& m, const Grid& g, const std::vector<Offset>& offsets) {
    Mask out(m.size(), 0);
    for (int iy = 0; iy < g.height; ++iy) {
        for (int ix = 0; ix < g.width; ++ix) {
            if (m[at(g, ix, iy)] == 0) continue;
            for (const Offset& o : offsets) {
                const int nx = ix + o.dx, ny = iy + o.dy;
                if (in_grid(g, nx, ny)) out[at(g, nx, ny)] = 1;
            }
        }
    }
    return out;
}

Mask invert(Mask m) {
    for (std::uint8_t& v : m) v = static_cast<std::uint8_t>(v == 0);
    return m;
}

Mask close_holes(const Mask& m, const Grid& g) {
    const std::vector<Offset> one = disk(1.0);
    return invert(grow(invert(grow(m, g, one)), g, one));
}

int cell_index(double v, double origin, double cell, int size) {
    // clamp: float rounding can put the max point one cell past the end
    return std::clamp(static_cast<int>(std::floor((v - origin) / cell)), 0, size - 1);
}

int cells_in(double length, double cell) {
    return std::max(1, static_cast<int>(std::lround(length / cell)));
}

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

std::vector<std::size_t> neighbour_indices(const ObjectGrid& grid, std::size_t index) {
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

long long doubled_signed_area(const std::vector<GridCorner>& ring) {
    long long doubled_area = 0;
    for (std::size_t index = 0; index < ring.size(); ++index) {
        const GridCorner& current = ring[index];
        const GridCorner& next = ring[(index + 1) % ring.size()];
        doubled_area += static_cast<long long>(current.x) * next.y -
                        static_cast<long long>(next.x) * current.y;
    }
    return doubled_area;
}

int length_between(GridCorner from, GridCorner to) {
    return std::abs(to.x - from.x) + std::abs(to.y - from.y);
}

bool is_same_corner(GridCorner first, GridCorner second) {
    return first.x == second.x && first.y == second.y;
}

}  // namespace vcortex
