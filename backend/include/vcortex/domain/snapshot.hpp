#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

// map at one moment, or one map.pcd; run_id and seq go into the Building to name its source
struct Snapshot {
    std::uint32_t run_id = 0;
    std::uint32_t seq = 0;
    // named by the source (ros header frame_id): another robot's map has another origin
    std::string frame_id;
    std::vector<LidarPoint> points;    // source map frame, z up, metres
};

// z kept though one floor needs only x, y: it tells the level and draws the walk in 3d
// no roll/pitch: point-lio already applied them and ingest lacks them, so they'd be faked
struct Pose {
    // source clock (ros time; bag time on replay), unix ns
    // int64: a double keeps only ~0.2 us at today's epoch
    std::int64_t time_ns = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;  // radians about z, counter-clockwise from x (ros rep 103)
};

// one pose per kPoseSpacing of travel, oldest first
using Trajectory = std::vector<Pose>;

// the model needs where the robot walked, not how often it stopped; a standing robot would
// otherwise pile up thousands of poses in one spot
inline constexpr double kPoseSpacing = 0.2;  // metres

// false when p is closer than kPoseSpacing to the last kept pose
inline bool append_pose(Trajectory& traj, const Pose& p) {
    if (!traj.empty() && std::hypot(p.position.x() - traj.back().position.x(),
                                    p.position.y() - traj.back().position.y()) < kPoseSpacing) {
        return false;
    }
    traj.push_back(p);
    return true;
}

}  // namespace vcortex
