#pragma once

#include <optional>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/geometry/vectors.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/grid.hpp"
#include "vcortex/model/floor/free_space.hpp"

namespace vcortex {

/// One way out of the room: walk along axis, towards larger (outward = +1) or smaller (-1)
/// coordinates. The walls met this way have a constant axis coordinate.
struct Direction {
    Axis axis = Axis::X;
    int outward = 1;
};

/// Where the wall face sits in one strip, a thin slice of the room across the wall
/// (aligned frame, metres).
struct WallFaceSample {
    double from = 0.0, to = 0.0;  // where the strip starts and ends along the wall
    std::optional<double> face;   // empty: nothing seen, or furniture hides it past max_reach
};

/// Cuts the room into strips across the walls met in direction and finds the wall face in
/// each one, walking out from the outermost room cell. Furniture in the middle of the room
/// is never taken for a wall. Samples come back in order of from.
std::vector<WallFaceSample> sample_wall_faces(const FreeSpace& space, const Grid& grid,
                                              const std::vector<LidarPoint>& aligned_points,
                                              const Plane& floor, double band_bottom_metres,
                                              double band_top_metres, Direction direction,
                                              const OutlineParams& parameters);

}  // namespace vcortex
