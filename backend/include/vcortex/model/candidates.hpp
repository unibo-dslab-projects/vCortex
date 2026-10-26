#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/settings.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

// Stage D (step 8): straight runs of occupied cells parallel to x or y.
// Walls and furniture faces both show up here; stage E sorts them apart.

enum class Axis { X, Y };  // X: the line has constant x (runs along y)

struct Candidate {
    Axis axis;
    double position;
    double from, to;
    std::size_t cells;
};

std::vector<Candidate> find_candidates(const Grid& grid, const CandidateParams& p);

}  // namespace vcortex
