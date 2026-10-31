#include "vcortex/model/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace vcortex {

namespace {

// 300 MB of counts and means (205 m square at 5 cm); a bigger grid means a stray point far
// away, not a building
constexpr std::size_t kMaxCells = std::size_t{1} << 24;

int cell_index(double v, double origin, double cell, int size) {
    // clamp: float rounding can put the max point one cell past the end
    return std::clamp(static_cast<int>(std::floor((v - origin) / cell)), 0, size - 1);
}

}  // namespace

Grid build_grid(const std::vector<Point>& aligned, const Plane& floor, double band_lo,
                double band_hi, double cell) {
    if (!(cell > 0.0)) {
        throw std::invalid_argument("build_grid: cell must be > 0");
    }
    Grid grid;
    grid.cell = cell;

    std::vector<Vec2> band;
    double min_x = std::numeric_limits<double>::max(), min_y = min_x;
    double max_x = std::numeric_limits<double>::lowest(), max_y = max_x;
    for (const Point& p : aligned) {
        const double h = height_above(floor, p);
        if (h >= band_lo && h < band_hi) {
            band.push_back({p.x, p.y});
            min_x = std::min(min_x, static_cast<double>(p.x));
            min_y = std::min(min_y, static_cast<double>(p.y));
            max_x = std::max(max_x, static_cast<double>(p.x));
            max_y = std::max(max_y, static_cast<double>(p.y));
        }
    }
    if (band.empty()) {
        return grid;
    }

    // snapped to a multiple of cell: the same wall lands in the same cells on every run
    grid.origin = {std::floor(min_x / cell) * cell, std::floor(min_y / cell) * cell};
    const double w = std::floor((max_x - grid.origin.x) / cell) + 1.0;
    const double h = std::floor((max_y - grid.origin.y) / cell) + 1.0;
    if (w * h > static_cast<double>(kMaxCells)) {
        throw std::runtime_error("build_grid: band spans " + std::to_string(max_x - min_x) + " x " +
                                 std::to_string(max_y - min_y) + " m, too large for one level");
    }
    grid.width = static_cast<int>(w);
    grid.height = static_cast<int>(h);
    const std::size_t cells =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    grid.column.assign(cells, 0);
    grid.mean.assign(cells, Vec2{});
    std::vector<std::uint32_t> n(cells, 0);  // column saturates, the mean must not

    for (const Vec2& v : band) {
        const int ix = cell_index(v.x, grid.origin.x, cell, grid.width);
        const int iy = cell_index(v.y, grid.origin.y, cell, grid.height);
        const std::size_t i = static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid.width) +
                              static_cast<std::size_t>(ix);
        std::uint16_t& count = grid.column[i];
        // saturate: a wrap to 0 would erase the densest cell
        if (count < std::numeric_limits<std::uint16_t>::max()) {
            ++count;
        }
        grid.mean[i].x += v.x;
        grid.mean[i].y += v.y;
        ++n[i];
    }
    for (std::size_t i = 0; i < cells; ++i) {
        if (n[i] > 0) {
            grid.mean[i].x /= n[i];
            grid.mean[i].y /= n[i];
        }
    }
    return grid;
}

std::vector<Vec2> band_cells(const Grid& grid, std::uint16_t min_column) {
    std::vector<Vec2> out;
    for (int iy = 0; iy < grid.height; ++iy) {
        for (int ix = 0; ix < grid.width; ++ix) {
            const std::size_t i = static_cast<std::size_t>(iy) *
                                      static_cast<std::size_t>(grid.width) +
                                  static_cast<std::size_t>(ix);
            if (grid.column[i] >= min_column) {
                out.push_back(grid.mean[i]);
            }
        }
    }
    return out;
}

}  // namespace vcortex
