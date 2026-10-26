#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// Stage F (step 10): split the free floor into rooms with a flood fill per trajectory pose.
std::vector<Room> find_rooms(const std::vector<Wall>& walls, const Grid& grid,
                             const Trajectory& aligned_traj, const RoomParams& p);

}  // namespace vcortex
