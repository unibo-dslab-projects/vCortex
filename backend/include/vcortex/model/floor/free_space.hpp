#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

/// Floor reached from the poses cell by cell, over seen floor that holds no low point, so the
/// floor under a desk counts. It is cut into rooms where a disk of door_radius no longer fits
/// between tall cells: a door between walls splits two rooms, an aisle between desks does not.
/// A room opens only from a pose; a room seen through a door belongs to the walked room next
/// to it until the robot walks in.
struct FreeSpace {
    std::vector<std::uint32_t> room_of_cell;  // same index as Grid::column; 0 = no room
    std::size_t room_count = 0;               // rooms are numbered 1..room_count
    double area = 0.0;                        // square metres, all rooms together
};

/// Aligned frame; same cells as grid, so later steps read both with one index. Recomputed from
/// scratch on each call, so room numbers can change between two calls on a growing map.
FreeSpace find_floor_rooms(const std::vector<LidarPoint>& aligned, const Plane& floor,
                           const Grid& grid, const Trajectory& trajectory,
                           const FreeSpaceParams& params);

}  // namespace vcortex
