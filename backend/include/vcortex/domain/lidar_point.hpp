#pragma once

#include <cstdint>

#include <Eigen/Core>

namespace vcortex {

// one voxel of the map as the lidar saw it
struct LidarPoint {
    // source map frame (camera_init on the go2): z up, metres
    // float, not double: lidar noise is cm, ingest sends float32, half the memory
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    // return strength as reported (0..255 on the go2's l1), float like the source;
    // the voxel's mean
    float intensity = 0.0F;
    // scans that hit the voxel: walls score high, noise and people passing by 1-2;
    // 0 when the source does not count (a pcd from point-lio's own save)
    std::uint32_t num_scan_seen = 0;
};

}  // namespace vcortex
