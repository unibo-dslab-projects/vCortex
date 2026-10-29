#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/candidates.hpp"
#include "vcortex/model/floor.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// wardrobe rule: a candidate with a parallel one close behind is an object front, the one
// behind is the wall; poses on both sides mean it stands inside a room, so not a wall
struct WallsAndObjects {
    std::vector<Wall> walls;
    std::vector<Object> objects;
};

// aligned frame in and out; walls get their ids here, since objects point at them
WallsAndObjects split_walls_objects(const std::vector<Candidate>& cands, const Grid& grid,
                                    const std::vector<Point>& aligned, const Plane& floor,
                                    double band_lo, double band_hi,
                                    const Trajectory& aligned_traj, const WallParams& p);

}  // namespace vcortex
