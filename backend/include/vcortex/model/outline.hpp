#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/settings.hpp"
#include "vcortex/model/wall_face_samples.hpp"

namespace vcortex {

// one straight piece of the room shell, aligned frame, metres; face is the room side of the
// wall, the only side a robot inside can see
struct OutlineSide {
    Direction direction;            // axis of the wall and the way out of the room
    double face = 0.0;              // constant coordinate of the face
    double from = 0.0, to = 0.0;    // span along the wall
    std::size_t voting_strips = 0;  // strips whose face decided the position
};

// samples of one direction in order of from (sample_wall_faces); sides come back in order of from
std::vector<OutlineSide> find_outline_sides(const std::vector<WallFaceSample>& samples,
                                            Direction direction, const OutlineParams& parameters);

// ends move to the face of a perpendicular side within corner_reach; corners hold the furniture
// that stopped the strip walk, so their strips have no face
void join_outline_corners(std::vector<OutlineSide>& sides, double corner_reach);

}  // namespace vcortex
