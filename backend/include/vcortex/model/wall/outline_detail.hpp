#pragma once

#include <optional>
#include <vector>

#include "vcortex/domain/settings.hpp"
#include "vcortex/model/wall/outline.hpp"

/// Steps of find_room_outlines, open to the tests: each takes plain numbers, so a case is a few
/// lines instead of a drawn room. Not for other callers.
namespace vcortex::detail {

/// The piece of wall one strip sees: from..to is where the strip lies along the wall, face is
/// where the wall face sits across it (aligned frame, metres). No face: the strip found no wall.
struct WallFaceSample {
    double from = 0.0, to = 0.0;
    std::optional<double> face;
};

/// The straight walls of one direction, from its samples in order along the wall.
/// Neighbouring samples with faces within segment_jump form one wall at their median face;
/// samples without a face (a door) neither split nor join. Then:
/// - walls of fewer than min_strips_per_segment samples are dropped (a stray strip);
/// - neighbouring walls within merge_distance become one;
/// - a wall at most dent_length long and at most dent_depth inward of a neighbour is covered by
///   that neighbour and does not vote (something stood in front of the wall).
std::vector<OutlineSide> find_outline_sides(const std::vector<WallFaceSample>& samples,
                                            Direction direction, const OutlineParams& parameters);

/// Stretches or trims both ends of every side to the corner it makes with a perpendicular side.
/// The seen part of a wall often stops short of its corners, where the view was blocked.
/// An end moves to the face of the nearest perpendicular side that faces out the way the end
/// heads, when each of the two reaches the other's face: short of it by up to
/// corner_max_extension, or past it by up to corner_overshoot. Otherwise the end stays.
/// The result does not depend on the order of sides.
void join_outline_corners(std::vector<OutlineSide>& sides, const OutlineParams& parameters);

}  // namespace vcortex::detail
