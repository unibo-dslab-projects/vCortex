#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/settings.hpp"
#include "vcortex/geometry/vectors.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// only lines along x or y: dominant_yaw squares the map first, so one pass per row and column
// is enough; walls and furniture faces both land here, find_walls tells them apart

/// A straight line of occupied cells that may be a wall face; aligned frame, metres.
struct Candidate {
    Axis axis = Axis::X;          // the normal: Axis::X is a line of constant x, running along y
    double position = 0.0;        // the constant coordinate: x for Axis::X, y for Axis::Y
    double from = 0.0, to = 0.0;  // extent along the line, from <= to
    // occupied only, not bridged: cells * cell / (to - from) = share really seen
    std::size_t cells = 0;
};

/// Finds every candidate line in the grid: first the Axis::X ones, then the Axis::Y ones, each
/// group sorted by position.
std::vector<Candidate> find_candidates(const Grid& grid, const CandidateParams& p);

}  // namespace vcortex
