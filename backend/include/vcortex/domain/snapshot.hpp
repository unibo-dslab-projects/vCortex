#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// The backend's map at one moment, or one map.pcd. run_id and seq travel into the Building, so a
// viewer can tell which map a result was computed from.
// The has_* flags exist because 0 is a real value for both fields: without them a missing field
// and a measured 0 would look the same.
struct Snapshot {
    std::uint32_t run_id = 0;
    std::uint32_t seq = 0;
    std::vector<Point> points;    // source map frame, z up, metres
    bool has_hits = false;        // old map.pcd files have no hits field: skip drop_transient
    bool has_intensity = false;
};

// Robot pose in the source's map frame: z up, metres. z is kept although one floor needs only x
// and y: it tells which level a pose belongs to, and the walk can be drawn in 3D.
// Roll and pitch are left out on purpose: Point-LIO already applied them to the points, the robot
// walks on floors, and the ingest protocol does not carry them, so they could only be faked.
struct Pose {
    // Source clock (ROS time; the bag's time on a replay), nanoseconds since the Unix epoch.
    // Integer, because a double holds only about 0.2 microseconds at today's epoch value.
    std::int64_t time_ns = 0;
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw = 0.0;  // radians about z, counter-clockwise from x (ROS REP 103)
};

// One pose every 0.2 m of travel, oldest first.
using Trajectory = std::vector<Pose>;

}  // namespace vcortex
