#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/candidates.hpp"
#include "vcortex/model/floor.hpp"
#include "vcortex/model/grid.hpp"
#include "vcortex/model/outline.hpp"
#include "vcortex/domain/point.hpp"

namespace vcortex {

// wardrobe rule: a candidate with a parallel one close behind is an object front, the one
// behind is the wall; poses on both sides mean it stands inside a room, so not a wall
struct WallsAndFurnitures {
    std::vector<Wall> walls;
    std::vector<Point> furniture_points;
    double shell_top_seen = 0.0;  // mean seen top of the shell walls, metres above the floor
};

// aligned frame in and out; walls get their ids here, since objects point at them
// the outline is the room shell: its sides are walls, candidates on or past it are dropped, and a
// candidate in front of it is an object front; an empty outline leaves the wardrobe rule alone
WallsAndFurnitures find_walls(const std::vector<Candidate>& cands,
                                    const std::vector<OutlineSide>& outline, const Grid& grid,
                                    const std::vector<Point>& aligned, const Plane& floor,
                                    double band_lo, double band_hi,
                                    const Trajectory& aligned_traj, const WallParams& p);

}  // namespace vcortex
