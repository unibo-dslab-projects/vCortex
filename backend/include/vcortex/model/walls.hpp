#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/candidates.hpp"
#include "vcortex/model/floor.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// A wardrobe against a wall looks like a wall from inside the room. So a candidate with another
// parallel candidate close behind it is the front of an object, and the one behind is the wall.
// A candidate with trajectory poses on both sides stands inside a room, so it is not a wall.
struct WallsAndObjects {
    std::vector<Wall> walls;
    std::vector<Object> objects;
};

WallsAndObjects split_walls_objects(const std::vector<Candidate>& cands, const Grid& grid,
                                    const std::vector<Point>& aligned, const Plane& floor,
                                    const Trajectory& aligned_traj, const WallParams& p);

}  // namespace vcortex
