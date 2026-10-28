#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/settings.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// Only runs parallel to x or y: the map is turned by dominant_yaw first, which reduces the search
// to one pass per grid row and column. Walls and furniture faces both show up here;
// split_walls_objects sorts them apart.

enum class Axis { X, Y };  // X: the line has constant x (runs along y)

// Aligned frame, metres.
struct Candidate {
    Axis axis = Axis::X;
    double position = 0.0;        // the constant coordinate: x for Axis::X, y for Axis::Y
    double from = 0.0, to = 0.0;  // span along the run, from <= to
    // Occupied cells only: cells bridged across a gap are not counted, so cells * cell / (to -
    // from) is the share of the run that was really seen.
    std::size_t cells = 0;
};

std::vector<Candidate> find_candidates(const Grid& grid, const CandidateParams& p);

}  // namespace vcortex
