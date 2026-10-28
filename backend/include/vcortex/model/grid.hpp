#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

// Seen from above. dominant_yaw, find_candidates and find_rooms share it, so the points are
// binned once per model run.
struct Grid {
    double cell;
    Vec2 origin;
    int width, height;
    std::vector<std::uint16_t> column;  // voxels per cell inside the height band
};

// Points in the aligned frame; band limits in metres above the floor.
Grid build_grid(const std::vector<Point>& aligned, const Plane& floor, double band_lo,
                double band_hi, double cell);

}  // namespace vcortex
