#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/candidates.hpp"
#include "vcortex/model/floor.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// Stage E (step 9): walls versus objects (the wardrobe rule). Aligned frame.
struct WallsAndObjects {
    std::vector<Wall> walls;
    std::vector<Object> objects;
};

WallsAndObjects split_walls_objects(const std::vector<Candidate>& cands, const Grid& grid,
                                    const std::vector<Point>& aligned, const Plane& floor,
                                    const Trajectory& aligned_traj, const WallParams& p);

}  // namespace vcortex
