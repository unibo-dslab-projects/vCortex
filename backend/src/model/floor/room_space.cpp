#include "vcortex/model/floor/room_space.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace vcortex {

namespace {
    constexpr double kSlack = 1e-9;

    using Mask = std::vector<std::uint8_t>;

    struct Offset {
        int dx, dy;
    };

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

    // grow, then shrink; shrink = grow the opposite, so the grid edge does not eat the mask
    Mask close_holes(const Mask& m, const Grid& g) {
        const std::vector<Offset> one = disk(1.0);
        return invert(grow(invert(grow(m, g, one)), g, one));
    }
} // namespace

RoomSpace compute_room_space(const std::vector<Point>& aligned, const Plane& floor, const Grid& grid, const Trajectory& trajectory, const RoomSpaceParams& params) {
    RoomSpace out;

    if (grid.width <= 0 || grid.height <= 0) return out;

    const std::size_t column_size = grid.column.size();
    out.inside.assign(column_size, 0);
    if (trajectory.empty()) return out;

    std::vector<std::uint16_t> low(column_size, 0);
    Mask floor_hit(column_size, 0);
    for (const Point& pt: aligned) {
        int ix = 0, iy = 0;
        if (!cell_of(grid, pt.x, pt.y, ix, iy)) continue;
        const double h = height_above(floor, pt);
        const std::size_t i = at(grid, ix, iy);
        if (h >= params.low_lo && h < params.low_hi && low[i] < std::numeric_limits<std::uint16_t>::max()) {
            ++low[i];
        }
        if (std::abs(h) < params.floor_band) floor_hit[i] = 1;
    }

    Mask blocked(column_size, 0);
    for (std::size_t i = 0; i < column_size; ++i) blocked[i] = low[i] >= params.min_points;

    // a missing voxel is not a hole; unseen floor blocks, so a live map never leaks into
    // a part not seen yet
    const Mask seen = close_holes(floor_hit, grid);
    const std::vector<Offset> coin = disk(params.coin_radius / grid.cell);
    Mask no_centre = grow(blocked, grid, coin);
    for (std::size_t i = 0; i < column_size; ++i) {
        if (seen[i] == 0) no_centre[i] = 1;
    }

    // 4 neighbours, as flood in rooms.cpp: diagonal steps slip between cells meeting at a corner
    Mask centres(column_size, 0);
    std::queue<std::pair<int, int>> todo;
    for (const Pose& pose : trajectory) {
        int ix = 0, iy = 0;
        if (!cell_of(grid, pose.x, pose.y, ix, iy)) continue;
        const std::size_t i = at(grid, ix, iy);
        if (no_centre[i] != 0 || centres[i] != 0) continue;
        centres[i] = 1;
        todo.push({ix, iy});
    }
    constexpr Offset kSteps[4] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!todo.empty()) {
        const auto [ix, iy] = todo.front();
        todo.pop();
        if (ix == 0 || iy == 0 || ix == grid.width - 1 || iy == grid.height - 1) out.leaked = true;
        for (const Offset& s : kSteps) {
            const int nx = ix + s.dx, ny = iy + s.dy;
            if (!in_grid(grid, nx, ny)) continue;
            const std::size_t n = at(grid, nx, ny);
            if (no_centre[n] != 0 || centres[n] != 0) continue;
            centres[n] = 1;
            todo.push({nx, ny});
        }
    }

    // growing back returns the radius, so straight walls lose nothing; corners lose r^2 (1 - pi/4)
    out.inside = grow(centres, grid, coin);
    std::size_t cells = 0;
    for (std::size_t i = 0; i < column_size; ++i) {
        if (blocked[i] != 0) out.inside[i] = 0;
        cells += out.inside[i];
    }
    out.area = static_cast<double>(cells) * grid.cell * grid.cell;
    return out;
}

} // namespace vcortex
