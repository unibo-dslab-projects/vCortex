#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/settings.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// only runs parallel to x or y: dominant_yaw squares the map first, so one pass per row and
// column; walls and furniture faces both land here, split_walls_objects sorts them

enum class Axis { X, Y };  // X: constant x, runs along y

// aligned frame, metres
struct Candidate {
    Axis axis = Axis::X;
    double position = 0.0;        // the constant coordinate: x for Axis::X, y for Axis::Y
    double from = 0.0, to = 0.0;  // span along the run, from <= to
    // occupied only, not bridged: cells * cell / (to - from) = share really seen
    std::size_t cells = 0;
};

std::vector<Candidate> find_candidates(const Grid& grid, const CandidateParams& p);

}  // namespace vcortex
