#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/wall/wall_face_samples.hpp"

namespace vcortex {

// one straight piece of the room shell, aligned frame, metres; face is the room side of the
// wall, the only side a robot inside can see
struct OutlineSide {
    Direction direction;            // axis of the wall and the way out of the room
    double face = 0.0;              // constant coordinate of the face
    double from = 0.0, to = 0.0;    // span along the wall
    std::size_t voting_strips = 0;  // strips whose face decided the position
    // cells of one cell width along the side with a point near the face, set by measure_outline_coverage
    std::size_t seen_cell_count = 0;
};

// samples of one direction in order of from (sample_wall_faces); sides come back in order of from
std::vector<OutlineSide> find_outline_sides(const std::vector<WallFaceSample>& samples,
                                            Direction direction, const OutlineParams& parameters);

// two perpendicular sides that turn towards each other are stretched until they cross: furniture
// hides most of a wall, so seen pieces stop metres short of the corner
void join_outline_corners(std::vector<OutlineSide>& sides, const OutlineParams& parameters);

// counts the cells along each side where a point of the band lies near its face; the other
// cells are drawn, not seen. Run after join_outline_corners, so corner stretches count as unseen.
void measure_outline_coverage(std::vector<OutlineSide>& sides,
                              const std::vector<Point>& aligned_points,
                              const Plane& aligned_floor, double band_bottom_metres,
                              double band_top_metres, double cell_size_metres,
                              double coverage_tolerance_metres);

}  // namespace vcortex
