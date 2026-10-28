#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

// Seen from above. Built twice per model run: on the raw points to feed dominant_yaw, then on the
// aligned points for find_candidates, split_walls_objects and find_rooms, because turning the
// points moves them into other cells.
// column[iy * width + ix] belongs to cell (ix, iy), whose lower-left corner is
// origin + (ix, iy) * cell. Row-major, so a grid row (constant y) is contiguous in memory.
// width and height are int, not size_t: neighbour walks step by -1 and need signed indices.
struct Grid {
    double cell = 0.0;                  // side length of one square cell
    Vec2 origin;
    int width = 0, height = 0;
    std::vector<std::uint16_t> column;  // points per cell inside the height band
};

// Points in the aligned frame; band limits in metres above the floor.
Grid build_grid(const std::vector<Point>& aligned, const Plane& floor, double band_lo,
                double band_hi, double cell);

}  // namespace vcortex
