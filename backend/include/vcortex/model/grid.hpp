#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/model/floor/floor.hpp"

namespace vcortex {

/// Top view of the band points: how many fall in each square cell, and where on average.
// built twice per run (raw for dominant_yaw, then aligned): turning moves points into other cells
struct Grid {
    double cell = 0.0;                                 // side of one square cell
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();  // lower-left corner of cell (0, 0)
    int width = 0, height = 0;          // int, not size_t: neighbour walks step by -1
    std::vector<std::uint16_t> column;  // row-major, column[iy * width + ix]; points in band
    // same index, mean x y of those points; centers sit on a lattice, and at 45 degrees whole
    // diagonals of centers land in one dominant_yaw bin (room_around scored 45 instead of 85.5)
    std::vector<Eigen::Vector2d> mean;
};

/// Counts the points between band_lo and band_hi, metres above the floor, per cell of a grid
/// that covers the whole cloud; throws when the grid would be too large for one level.
Grid build_grid(const std::vector<LidarPoint>& aligned, const Plane& floor, double band_lo,
                double band_hi, double cell);

/// Returns the mean point of every cell holding at least min_column points.
// min_column keeps one-voxel specks from voting in dominant_yaw
std::vector<Eigen::Vector2d> band_cells(const Grid& grid, std::uint16_t min_column);

}  // namespace vcortex
