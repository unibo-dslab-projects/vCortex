#pragma once

#include <cstdint>

namespace vcortex {

// source map frame (camera_init on the go2): z up, metres
// float, not double: lidar noise is cm, ingest sends float32, half the memory
struct Point {
    float x = 0.0F, y = 0.0F, z = 0.0F;
    // return strength as reported (0..255 on the go2's l1), float like the source;
    // first reading per voxel only, so noisy
    float intensity = 0.0F;
    std::uint16_t hits = 0;  // scans that hit this voxel; 0 = unknown, not "never seen"
};

struct Vec2 {
    double x = 0.0, y = 0.0;
};

}  // namespace vcortex
