#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

// Occupancy grid seen from above, shared by stages C, D and F.
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
