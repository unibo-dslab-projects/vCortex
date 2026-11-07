#pragma once

#include <cstdint>

namespace vcortex {

// source map frame (camera_init on the go2): z up, metres
// float, not double: lidar noise is cm, ingest sends float32, half the memory
struct Point {
    float x = 0.0F, y = 0.0F, z = 0.0F;
    // return strength as reported (0..255 on the go2's l1), float like the source;
    // the voxel's mean
    float intensity = 0.0F;
    // scans that hit the voxel: walls score high, noise and people passing by 1-2;
    // 0 when the source does not count (a pcd from point-lio's own save)
    std::uint32_t num_scan_seen = 0;
};

struct Vec2 {
    double x = 0.0, y = 0.0;
};

}  // namespace vcortex
