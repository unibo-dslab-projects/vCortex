#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/wall/candidates.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/grid.hpp"
#include "vcortex/model/wall/outline.hpp"
#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

/// What find_walls returns: the walls, and the band points that belong to no wall.
struct WallsAndFurniture {
    std::vector<Wall> walls;                   // ids "wall-1", "wall-2", ... in sorted order
    std::vector<LidarPoint> furniture_points;  // inside the room shell, far from every wall
    double shell_top_seen = 0.0;  // mean seen top of the shell walls, metres above the floor
};

/// Tells walls from furniture fronts among the candidates and returns the walls, joined along
/// their lines and closed at corners; aligned frame in and out, band limits above the floor.
WallsAndFurniture find_walls(const std::vector<Candidate>& cands,
                                    const std::vector<OutlineSide>& outline, const Grid& grid,
                                    const std::vector<LidarPoint>& aligned, const Plane& floor,
                                    double band_lo, double band_hi,
                                    const Trajectory& aligned_traj, const WallParams& p);

}  // namespace vcortex
