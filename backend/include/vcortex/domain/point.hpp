#pragma once

#include <cstdint>

namespace vcortex {

// One map point in the source's map frame (camera_init for the Go2): z up, metres.
struct Point {
    float x, y, z;
    std::uint16_t hits;  // number of scans that hit this voxel; 0 means "unknown"
};

struct Vec2 {
    double x, y;
};

}  // namespace vcortex
