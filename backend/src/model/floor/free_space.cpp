#include "vcortex/model/floor/free_space.hpp"
#include "vcortex/geometry/grids.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace vcortex {

namespace {

/// Struct for storing the results of cell classification, including blocked and
/// floor hit cells.
struct ClassifiedCells {
  std::vector<std::uint8_t> blocked;   // enough points in the low band: furniture or wall
  std::vector<std::uint8_t> floor_hit; // at least one point on the floor
  std::vector<std::uint8_t> tall;      // a point at door height or above: wall, not desk
};

/// Marks each cell as blocked (enough points in the low band, meaning an object
/// is in) and as floor hit (a point on the floor).
ClassifiedCells classify_cells(const std::vector<LidarPoint> &aligned, const Plane &floor,
                                const Grid &grid, const FreeSpaceParams &params) {
    const std::size_t cell_count = grid.column.size();
    std::vector<std::uint16_t> low_point_counts(cell_count, 0);
    ClassifiedCells cells;
    cells.floor_hit.assign(cell_count, 0);
    cells.tall.assign(cell_count, 0);
    // count low band points and mark floor hit cells
    for (const LidarPoint &point : aligned) {
        int cell_x = 0, cell_y = 0;
        if (!cell_of(grid, point.position.x(), point.position.y(), cell_x, cell_y)) continue;
        const double height = height_above(floor, point);
        const std::size_t cell = at(grid, cell_x, cell_y);
        if (height >= params.low_lo && height < params.low_hi &&
            low_point_counts[cell] < std::numeric_limits<std::uint16_t>::max()) {
          ++low_point_counts[cell];
        }
        if (std::abs(height) < params.floor_band) cells.floor_hit[cell] = 1;
        if (height >= params.tall_from) cells.tall[cell] = 1;
    }
    // mark blocked cells based on low band point count
    cells.blocked.assign(cell_count, 0);
    for (std::size_t i = 0; i < cell_count; ++i) {
        cells.blocked[i] = low_point_counts[i] >= params.min_points;
    }
    return cells;
}

/// Returns the cells where the disk's center cannot go: within its radius of a
/// blocked cell, or on floor never seen.
std::vector<std::uint8_t>
find_blocked_centers(std::vector<std::uint8_t> obstacles, std::vector<std::uint8_t> seen,
                        const Grid &grid, const std::vector<Eigen::Vector2i> &clearance_disk) {
    // a missing voxel is not a hole; unseen floor blocks, so a live map never
    // leaks into a part not seen yet
    std::vector<std::uint8_t> blocked_centers = grow(obstacles, grid, clearance_disk);
    for (std::size_t i = 0; i < blocked_centers.size(); ++i) {
        if (!seen[i]) blocked_centers[i] = 1;
    }
    return blocked_centers;
}

/// Spreads the disk's center from every pose to the cells it can reach, noting
/// if it reached the grid edge.
std::vector<std::uint8_t> spread_room_labels(const Trajectory &trajectory,
                   const std::vector<std::uint8_t> &blocked_centers, const Grid &grid) {
    std::vector<std::uint8_t> visited(blocked_centers.size(), 0);
    std::queue<std::pair<int, int>> cells_to_visit;
    // visit each pose and mark it reachable
    for (const Pose &pose : trajectory) {
        int cell_x = 0, cell_y = 0;
        if (!cell_of(grid, pose.position.x(), pose.position.y(), cell_x, cell_y))
            continue;
        const std::size_t cell = at(grid, cell_x, cell_y);
        if (blocked_centers[cell] || visited[cell])
            continue;
        visited[cell] = 1;
        cells_to_visit.push({cell_x, cell_y});
    }
    // flood fill from the reachable cells through bfs algorithm
    const Eigen::Vector2i directions[4] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!cells_to_visit.empty()) {
        const auto [cell_x, cell_y] = cells_to_visit.front();
        cells_to_visit.pop();
        if (cell_x == 0 || cell_y == 0 || cell_x == grid.width - 1 || cell_y == grid.height - 1) {
            continue;
        }
        for (const Eigen::Vector2i &direction : directions) {
            const int neighbour_x = cell_x + direction.x(),
                    neighbour_y = cell_y + direction.y();
            if (!is_in_grid(grid, neighbour_x, neighbour_y))
                continue;
            const std::size_t neighbour = at(grid, neighbour_x, neighbour_y);
            if (blocked_centers[neighbour] || visited[neighbour])
                continue;
            visited[neighbour] = 1;
            cells_to_visit.push({neighbour_x, neighbour_y});
        }
    }
    return visited;
}

/// Grows the reached space of the missing cells, missing due to being blocked
/// by not enough space to make the clearance disk fit.
FreeSpace expand_unreached_space(const std::vector<std::uint8_t> &free,
                       const std::vector<std::uint8_t> &blocked, const Grid &grid,
                       const std::vector<Eigen::Vector2i> &clearance_disk) {
    FreeSpace space;
    // growing back returns the radius, so straight walls lose nothing; corners
    // lose r^2 (1 - pi/4)
    space.inside = grow(free, grid, clearance_disk);
    std::size_t inside_cell_count = 0;
    for (std::size_t i = 0; i < space.inside.size(); ++i) {
        if (blocked[i])
            space.inside[i] = 0;
        inside_cell_count += space.inside[i];
    }
    space.area = static_cast<double>(inside_cell_count) * grid.cell * grid.cell;
    return space;
}

} // namespace

FreeSpace compute_free_space(const std::vector<LidarPoint> &aligned, const Plane &floor, const Grid &grid,
                             const Trajectory &trajectory, const FreeSpaceParams &params) {
    FreeSpace space;
    if (grid.width <= 0 || grid.height <= 0) return space;
    space.inside.assign(grid.column.size(), 0);
    if (trajectory.empty()) return space;

    const std::vector<Eigen::Vector2i> clearance_disk = disk(params.clearance_radius / grid.cell);
    const ClassifiedCells cells = classify_cells(aligned, floor, grid, params);
    const std::vector<std::uint8_t> blocked_centers = find_blocked_centers(
        cells.blocked, cells.floor_hit, grid, clearance_disk);
    const std::vector<std::uint8_t> flood = spread_room_labels(trajectory, blocked_centers, grid);
    return expand_unreached_space(flood, cells.blocked, grid, clearance_disk);
}

} // namespace vcortex
