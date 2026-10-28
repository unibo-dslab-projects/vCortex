#pragma once

#include <cstdint>

namespace vcortex {

// In the source's map frame (camera_init for the Go2): z up, metres.
// float, not double: the lidar noise is centimetres, and float matches the ingest record
// (x y z float32), so a frame decodes without conversion and the map takes half the memory.
struct Point {
    float x = 0.0F, y = 0.0F, z = 0.0F;
    // Return strength as the lidar reports it (0..255 on the Go2's L1), float like the source.
    // One reading per voxel, the first that landed in it, so it is noisy.
    float intensity = 0.0F;
    std::uint16_t hits = 0;  // scans that hit this voxel; 0 means unknown, not "never seen"
};

struct Vec2 {
    double x = 0.0, y = 0.0;
};

}  // namespace vcortex
