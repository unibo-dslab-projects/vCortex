#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

// top view, built twice per run (raw for dominant_yaw, then aligned): turning moves points
// into other cells
struct Grid {
    double cell = 0.0;                  // side of one square cell
    Vec2 origin;                        // lower-left corner of cell (0, 0)
    int width = 0, height = 0;          // int, not size_t: neighbour walks step by -1
    std::vector<std::uint16_t> column;  // row-major, column[iy * width + ix]; points in band
    // same index, mean x y of those points; centres sit on a lattice, and at 45 degrees whole
    // diagonals of centres land in one dominant_yaw bin (room_around scored 45 instead of 85.5)
    std::vector<Vec2> mean;
};

// band limits in metres above the floor
Grid build_grid(const std::vector<Point>& aligned, const Plane& floor, double band_lo,
                double band_hi, double cell);

// one entry per cell at its points' mean; same min_column as find_candidates, so one-voxel
// specks do not vote
std::vector<Vec2> band_cells(const Grid& grid, std::uint16_t min_column);

}  // namespace vcortex
