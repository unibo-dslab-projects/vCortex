#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/settings.hpp"
#include "vcortex/geometry/vectors.hpp"
#include "vcortex/model/floor/free_space.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {

/// One way out of the room: walk along axis, towards larger (outward = +1) or smaller (-1)
/// coordinates. The walls met this way have a constant axis coordinate.
struct Direction {
    Axis axis = Axis::X;
    int outward = 1;
};

/// One straight wall of a room outline, seen from inside (aligned frame, metres).
struct OutlineSide {
    Direction direction;          // the axis the wall is across, and which way is out
    double face = 0.0;            // where the wall's inner face sits on that axis
    double from = 0.0, to = 0.0;  // where the wall starts and ends along it
};

/// One outline per room of space, at index room - 1: the straight walls around the room,
/// measured from inside. Each wall comes from rows of cells walked out of the room until a wall,
/// so a wall seen from one side only is still found. Corners hidden by furniture are joined.
/// The points of grid vote, so build it from outline.band_lo; space must come from the same
/// grid, so one index is one cell in both. A wall between two rooms shows up in both outlines.
std::vector<std::vector<OutlineSide>> find_room_outlines(const FreeSpace& space, const Grid& grid,
                                                         const OutlineParams& parameters);

}  // namespace vcortex
