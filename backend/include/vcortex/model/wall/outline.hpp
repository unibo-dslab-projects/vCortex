#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/wall/face_samples.hpp"

namespace vcortex {

/// One straight wall of the room outline, seen from inside (aligned frame, metres).
struct OutlineSide {
    Direction direction;            // the axis the wall is across, and which way is out
    double face = 0.0;              // where the wall's inner face sits on that axis
    double from = 0.0, to = 0.0;    // where the wall starts and ends along it
    std::size_t voting_strips = 0;  // how many strips agreed on face
    // cells along the wall with a point on the face; the rest is drawn, not seen
    std::size_t seen_cell_count = 0;
};

/// Turns the face samples of one direction into straight walls: neighbouring samples at the
/// same depth join, and short dents made by furniture against the wall are bridged.
/// Samples and walls are both ordered by from.
std::vector<OutlineSide> find_outline_sides(const std::vector<WallFaceSample>& samples,
                                            Direction direction, const OutlineParams& parameters);

/// Stretches or trims the ends of two perpendicular walls until they meet in a corner.
/// Furniture hides corners, so the seen parts of a wall stop short of them.
void join_outline_corners(std::vector<OutlineSide>& sides, const OutlineParams& parameters);

/// Sets seen_cell_count on each wall. Call after join_outline_corners, so stretched ends
/// count as not seen.
void measure_outline_coverage(std::vector<OutlineSide>& sides,
                              const std::vector<LidarPoint>& aligned_points,
                              const Plane& aligned_floor, double band_bottom_metres,
                              double band_top_metres, double cell_size_metres,
                              double coverage_tolerance_metres);

}  // namespace vcortex
