#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// The backend's map at one moment, or one map.pcd.
struct Snapshot {
    std::uint32_t run_id;
    std::uint32_t seq;
    std::vector<Point> points;  // source map frame, z up, metres
    bool has_hits;              // false for old map.pcd files without the field
};

// Robot position seen from above.
struct Pose2 {
    double x, y;
};

// Decimated: one pose every 0.2 m of travel.
using Trajectory = std::vector<Pose2>;

}  // namespace vcortex
