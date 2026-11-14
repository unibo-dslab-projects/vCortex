#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "vcortex/model/object/grid.hpp"

namespace vcortex {

/// Cell corners of the outer ring of one object, counter-clockwise, corners only; corner
/// (x, y) sits at origin + (x, y) * cell. Holes are left out, so a desk seen only at its rim
/// comes back whole.
std::vector<Eigen::Vector2i> trace_outline(const ObjectGrid& grid,
                                           const std::vector<std::uint32_t>& labels,
                                           std::uint32_t object_id);

/// Straighten stair steps shorter than min_segment_cells: the short edge goes and its two
/// neighbours join on one line. Spikes and notches (neighbours running opposite ways) stay.
void drop_small_segments(std::vector<Eigen::Vector2i>& outline, int min_segment_cells);

}  // namespace vcortex
