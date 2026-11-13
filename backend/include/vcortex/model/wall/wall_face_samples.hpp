#pragma once

#include <optional>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/model/wall/candidates.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/grid.hpp"
#include "vcortex/model/floor/room_space.hpp"

namespace vcortex {

// one way out of the room: axis of the walls met, and +1 / -1 for walking towards larger /
// smaller coordinates
struct Direction {
    Axis axis = Axis::X;  // X: walk along x, meet walls of constant x
    int outward = 1;
};

// aligned frame, metres
struct WallFaceSample {
    double from = 0.0, to = 0.0;  // span of the strip along the wall
    std::optional<double> face;   // empty: furniture past max_reach, or nothing seen
};

// one sample per strip, in order of from; walks out from the outermost room cell of each strip, so furniture
// in the middle of the room is never asked
std::vector<WallFaceSample> sample_wall_faces(const RoomSpace& space, const Grid& grid,
                                        const std::vector<LidarPoint>& aligned, const Plane& floor,
                                        double low_lo, double band_hi, Direction d,
                                        const OutlineParams& p);

}  // namespace vcortex
