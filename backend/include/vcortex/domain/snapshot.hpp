#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

struct Snapshot {
    std::uint32_t run_id = 0;
    std::uint32_t seq = 0;
    std::string frame_id;
    std::vector<LidarPoint> points;
};

struct Pose {
    std::int64_t time_ns = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;  // radians about z, counter-clockwise from x (ros rep 103)
};

using Trajectory = std::vector<Pose>;

// min distance between poses in the trajectory in meters
inline constexpr double kPoseSpacing = 0.2;

/// Appends a pose to the trajectory if it is far enough from the last pose
inline bool append_pose(Trajectory& traj, const Pose& p) {
    if (!traj.empty() && std::hypot(p.position.x() - traj.back().position.x(),
                                    p.position.y() - traj.back().position.y()) < kPoseSpacing) {
        return false;
    }
    traj.push_back(p);
    return true;
}

}  // namespace vcortex
