#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// Flood fill from each trajectory pose: the robot only walks on free floor, so every pose is a
// seed that is surely inside a room.
std::vector<Room> find_rooms(const std::vector<Wall>& walls, const Grid& grid,
                             const Trajectory& aligned_traj, const RoomParams& p);

}  // namespace vcortex
