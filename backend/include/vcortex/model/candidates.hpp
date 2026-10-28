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

struct Candidate {
    Axis axis;
    double position;
    double from, to;
    std::size_t cells;
};

std::vector<Candidate> find_candidates(const Grid& grid, const CandidateParams& p);

}  // namespace vcortex
