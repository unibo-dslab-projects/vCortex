#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// map at one moment, or one map.pcd; run_id and seq go into the Building to name its source
// has_* flags: 0 is a real value, so a missing field must not look like a measured 0
struct Snapshot {
    std::uint32_t run_id = 0;
    std::uint32_t seq = 0;
    // named by the source (ros header frame_id): another robot's map has another origin
    std::string frame_id;
    std::vector<Point> points;    // source map frame, z up, metres
    bool has_hits = false;        // old map.pcd files lack hits: skip drop_transient
    bool has_intensity = false;
};

// z kept though one floor needs only x, y: it tells the level and draws the walk in 3d
// no roll/pitch: point-lio already applied them and ingest lacks them, so they'd be faked
struct Pose {
    // source clock (ros time; bag time on replay), unix ns
    // int64: a double keeps only ~0.2 us at today's epoch
    std::int64_t time_ns = 0;
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw = 0.0;  // radians about z, counter-clockwise from x (ros rep 103)
};

// one pose per 0.2 m of travel, oldest first
using Trajectory = std::vector<Pose>;

}  // namespace vcortex
