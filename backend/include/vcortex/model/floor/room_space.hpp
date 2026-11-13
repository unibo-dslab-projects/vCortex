#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// floor a disk of coin_radius reaches from the poses without touching a low point: walls
// with gaps under twice the radius keep it in, and what lies beyond them never counts
struct RoomSpace {
    std::vector<std::uint8_t> inside;  // same index as Grid::column; 1 = room space
    double area = 0.0;                 // square metres
    bool leaked = false;               // reached the grid edge: the shell is open somewhere
};

// aligned frame; same cells as grid, so later steps read both with one index
RoomSpace compute_room_space(const std::vector<LidarPoint>& aligned, const Plane& floor,
                             const Grid& grid, const Trajectory& trajectory,
                             const RoomSpaceParams& params);

}  // namespace vcortex
