#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// flood fill from each pose: the robot walks only on free floor, so each pose is in a room
// aligned frame in and out; door_max is WallParams::door_max, gaps up to it are closed so
// rooms do not flow into each other through doors
std::vector<Room> find_rooms(const std::vector<Wall>& walls, const Grid& grid,
                             const Trajectory& aligned_traj, double door_max,
                             const RoomParams& p);

}  // namespace vcortex
