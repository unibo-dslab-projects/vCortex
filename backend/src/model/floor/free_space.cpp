#include "vcortex/model/floor/free_space.hpp"
#include "vcortex/geometry/grids.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace vcortex {

namespace {

/// What the points say about each cell, one flag per cell.
struct ClassifiedCells {
    std::vector<std::uint32_t> blocked;    // enough points in the low band: furniture or wall
    std::vector<std::uint32_t> floor_hit;  // at least one point on the floor
    std::vector<std::uint32_t> tall;       // a point at door height or above: wall, not desk
};

/// Marks each cell as blocked (enough points in the low band, meaning an object is in),
/// as floor hit (a point on the floor) and as tall (a point at tall_from or above).
ClassifiedCells classify_cells(const std::vector<LidarPoint>& aligned, const Plane& floor,
                               const Grid& grid, const FreeSpaceParams& params) {
    const std::size_t cell_count = grid.column.size();
    std::vector<std::uint16_t> low_point_counts(cell_count, 0);
    ClassifiedCells cells;
    cells.floor_hit.assign(cell_count, 0);
    cells.tall.assign(cell_count, 0);
    for (const LidarPoint& point : aligned) {
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
    cells.blocked.assign(cell_count, 0);
    for (std::size_t i = 0; i < cell_count; ++i) {
        cells.blocked[i] = low_point_counts[i] >= params.min_points;
    }
    return cells;
}

/// True when a disk centred on (x, y) covers no obstacle cell; cells past the grid edge are no
/// obstacle.
bool disk_fits(int x, int y, const std::vector<Eigen::Vector2i>& disk_offsets,
               const std::vector<std::uint32_t>& obstacles, const Grid& grid) {
    for (const Eigen::Vector2i& offset : disk_offsets) {
        const int covered_x = x + offset.x(), covered_y = y + offset.y();
        if (is_in_grid(grid, covered_x, covered_y) && obstacles[at(grid, covered_x, covered_y)]) {
            return false;
        }
    }
    return true;
}

/// Breadth-first search: copies each queued cell's label onto its four neighbours that are
/// allowed, still unlabelled, and where the disk fits clear of the obstacles. Four neighbours,
/// so two areas touching only at a corner stay apart. Any non-zero cell of allowed is allowed.
void spread_labels(std::queue<std::pair<int, int>> cells_to_visit,
                   std::vector<std::uint32_t>& label_of_cell,
                   const std::vector<Eigen::Vector2i>& disk_offsets,
                   const std::vector<std::uint32_t>& obstacles,
                   const std::vector<std::uint32_t>& allowed, const Grid& grid) {
    const Eigen::Vector2i directions[4] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!cells_to_visit.empty()) {
        const auto [cell_x, cell_y] = cells_to_visit.front();
        cells_to_visit.pop();
        const std::uint32_t label = label_of_cell[at(grid, cell_x, cell_y)];
        for (const Eigen::Vector2i& direction : directions) {
            const int neighbour_x = cell_x + direction.x(), neighbour_y = cell_y + direction.y();
            if (!is_in_grid(grid, neighbour_x, neighbour_y)) continue;
            const std::size_t neighbour = at(grid, neighbour_x, neighbour_y);
            if (!allowed[neighbour] || label_of_cell[neighbour] != 0) continue;
            if (!disk_fits(neighbour_x, neighbour_y, disk_offsets, obstacles, grid)) continue;
            label_of_cell[neighbour] = label;
            cells_to_visit.push({neighbour_x, neighbour_y});
        }
    }
}

/// Numbers the areas reached from the poses, from 1; 0 = not reached. A pose on an allowed cell
/// where the disk fits, and not reached yet, opens a new area, which spreads as spread_labels
/// does. With a one-cell disk and the low obstacles this finds the floor; with the door disk and
/// the tall cells it finds the room cores, split at doors.
std::vector<std::uint32_t> label_area_reached(const Trajectory& trajectory,
                                              const std::vector<Eigen::Vector2i>& disk_offsets,
                                              const std::vector<std::uint32_t>& obstacles,
                                              const std::vector<std::uint32_t>& allowed,
                                              const Grid& grid) {
    std::vector<std::uint32_t> label_of_cell(allowed.size(), 0);
    std::uint32_t next_label = 1;
    for (const Pose& pose : trajectory) {
        int cell_x = 0, cell_y = 0;
        if (!cell_of(grid, pose.position.x(), pose.position.y(), cell_x, cell_y)) continue;
        const std::size_t cell = at(grid, cell_x, cell_y);
        if (!allowed[cell] || label_of_cell[cell] != 0) continue;
        if (!disk_fits(cell_x, cell_y, disk_offsets, obstacles, grid)) continue;
        label_of_cell[cell] = next_label++;
        std::queue<std::pair<int, int>> cells_to_visit;
        cells_to_visit.push({cell_x, cell_y});
        spread_labels(cells_to_visit, label_of_cell, disk_offsets, obstacles, allowed, grid);
    }
    return label_of_cell;
}

/// Grows all rooms together, cell by cell, over the rest of the floor, so two rooms meet
/// halfway through the door between them.
void expand_rooms_labels(std::vector<std::uint32_t>& room_of_cell, const std::vector<std::uint32_t>& floor_areas,
                         const std::vector<std::uint32_t>& blocked, const Grid& grid) {
    std::queue<std::pair<int, int>> room_cells;
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (room_of_cell[at(grid, x, y)]) room_cells.push({x, y});
        }
    }
    spread_labels(room_cells, room_of_cell, disk(0.0), blocked, floor_areas, grid);
}

}  // namespace

FreeSpace find_floor_rooms(const std::vector<LidarPoint>& aligned, const Plane& floor,
                           const Grid& grid, const Trajectory& trajectory, const FreeSpaceParams& params) {
    FreeSpace space;
    if (grid.width <= 0 || grid.height <= 0) return space;
    space.room_of_cell.assign(grid.column.size(), 0);
    if (aligned.empty() || trajectory.empty()) return space;

    const ClassifiedCells cells = classify_cells(aligned, floor, grid, params);
    const std::vector<std::uint32_t> seen = close_holes(cells.floor_hit, grid);
    const std::vector<Eigen::Vector2i> door_disk = disk(params.door_radius / grid.cell);
    const std::vector<std::uint32_t> floor_areas =
        label_area_reached(trajectory, disk(0.0), cells.blocked, seen, grid);
    space.room_of_cell = label_area_reached(trajectory, door_disk, cells.tall, floor_areas, grid);
    space.room_count = *std::max_element(space.room_of_cell.begin(), space.room_of_cell.end());
    expand_rooms_labels(space.room_of_cell, floor_areas, cells.blocked, grid);
    // TODO: tell a closed room from one still open toward floor not seen yet

    const auto room_cell_count = std::count_if(space.room_of_cell.begin(), space.room_of_cell.end(),
                                               [](std::uint32_t room) { return room != 0; });
    space.area = static_cast<double>(room_cell_count) * grid.cell * grid.cell;
    return space;
}

}  // namespace vcortex
