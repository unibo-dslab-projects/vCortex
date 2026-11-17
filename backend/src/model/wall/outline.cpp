#include "vcortex/model/wall/outline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

#include "vcortex/geometry/grids.hpp"
#include "vcortex/model/median.hpp"
#include "vcortex/model/wall/outline_detail.hpp"

namespace vcortex {

namespace {

constexpr double kEpsilon = 1e-9;

// the four ways out of a room, one per wall direction once the map is squared
constexpr Direction kWaysOut[4] = {{Axis::X, 1}, {Axis::X, -1}, {Axis::Y, 1}, {Axis::Y, -1}};


/// A band of row_count grid rows from first_row, walked out of the room together.
/// Walk cells run along the walk direction, from 0 to walk_cell_count; rows run along the wall.
/// For an X direction a walk cell is a grid x index and a row a grid y index; for Y, swapped.
struct Strip {
    Direction direction;
    int first_row = 0, row_count = 0, walk_cell_count = 0;
};

/// Grid index of the cell at walk_cell and row of the strip.
std::size_t cell_at(const Grid& grid, const Strip& strip, int walk_cell, int row) {
    return strip.direction.axis == Axis::X ? at(grid, walk_cell, row) : at(grid, row, walk_cell);
}

/// Whether any row of the strip at walk_cell belongs to room.
bool has_room_cell(const FreeSpace& space, std::uint32_t room, const Grid& grid, const Strip& strip,
                   int walk_cell) {
    for (int row = strip.first_row; row < strip.first_row + strip.row_count; ++row) {
        if (space.room_of_cell[cell_at(grid, strip, walk_cell, row)] == room) return true;
    }
    return false;
}

/// Walk cell of the room cell farthest out in the strip, or -1 when the strip misses the room.
/// The search starts at the outer edge of the grid and goes in, so the first room cell met is
/// the farthest out, and holes in the room floor (under a table) never stop it early.
int find_outermost_room_cell(const FreeSpace& space, std::uint32_t room, const Grid& grid,
                             const Strip& strip) {
    for (int step = 0; step < strip.walk_cell_count; ++step) {
        const int walk_cell = strip.direction.outward > 0 ? strip.walk_cell_count - 1 - step : step;
        if (has_room_cell(space, room, grid, strip, walk_cell)) return walk_cell;
    }
    return -1;
}

/// Whether any row of the strip at walk_cell holds a band point; false outside the grid.
/// One row is enough: the rows fill each other's holes on a sparsely seen wall.
bool has_band_point(const Grid& band_grid, const Strip& strip, int walk_cell) {
    if (walk_cell < 0 || walk_cell >= strip.walk_cell_count) return false;
    for (int row = strip.first_row; row < strip.first_row + strip.row_count; ++row) {
        if (band_grid.column[cell_at(band_grid, strip, walk_cell, row)] > 0) return true;
    }
    return false;
}

/// Walk cell of the wall, walking out from outermost_cell: the last cell with band points
/// before max_gap_cells empty cells in a row. -1 when max_reach_cells runs out first, or when
/// nothing is met. Furniture against the wall leaves shorter gaps, so the walk goes through
/// it to the wall behind.
int find_wall_cell(const Grid& band_grid, const Strip& strip, int outermost_cell,
                   int max_reach_cells, int max_gap_cells) {
    int last_occupied_step = 0, empty_cells = 0;
    for (int step = 1; step <= max_reach_cells; ++step) {
        const int walk_cell = outermost_cell + strip.direction.outward * step;
        if (has_band_point(band_grid, strip, walk_cell)) {
            last_occupied_step = step;
            empty_cells = 0;
        } else if (++empty_cells >= max_gap_cells) {
            return last_occupied_step > 0
                       ? outermost_cell + strip.direction.outward * last_occupied_step
                       : -1;
        }
    }
    return -1;
}

/// Position of the wall face on the walk axis (metres): the median of the band point means in
/// the face_depth_cells cells up to wall_cell, over all rows of the strip. The lidar sees only
/// the inner face of a wall, so those points are the face; the narrow window keeps furniture in
/// front of it out. Empty when the window holds no point.
std::optional<double> find_face(const Grid& band_grid, const Strip& strip, int wall_cell,
                                int face_depth_cells) {
    std::vector<double> positions;
    for (int depth = 0; depth < face_depth_cells; ++depth) {
        const int walk_cell = wall_cell - strip.direction.outward * depth;
        if (walk_cell < 0 || walk_cell >= strip.walk_cell_count) continue;
        for (int row = strip.first_row; row < strip.first_row + strip.row_count; ++row) {
            const std::size_t cell = cell_at(band_grid, strip, walk_cell, row);
            if (!band_grid.column[cell]) continue;
            const Eigen::Vector2d& mean = band_grid.mean[cell];
            positions.push_back(strip.direction.axis == Axis::X ? mean.x() : mean.y());
        }
    }
    if (positions.empty()) return std::nullopt;
    return median(std::move(positions));
}

/// The wall faces of room seen in direction, as one WallFaceSample per strip, in order along
/// the wall. The grid is cut into strips of parameters.strip across the walls; each strip that
/// touches the room gives one sample, with or without a face. Strips that miss the room give
/// none. Neighbouring samples are joined into outline sides afterwards.
std::vector<detail::WallFaceSample> sample_wall_faces(const FreeSpace& space, std::uint32_t room,
                                              const Grid& band_grid, Direction direction,
                                              const OutlineParams& parameters) {
    const int row_count = direction.axis == Axis::X ? band_grid.height : band_grid.width;
    const int walk_cell_count = direction.axis == Axis::X ? band_grid.width : band_grid.height;
    const double along_start =
        direction.axis == Axis::X ? band_grid.origin.y() : band_grid.origin.x();
    // from meters to cells
    const int cells_per_strip = num_cells_in_len(parameters.strip, band_grid.cell);
    const int max_gap_cells = num_cells_in_len(parameters.max_gap, band_grid.cell);
    const int max_reach_cells = num_cells_in_len(parameters.max_reach, band_grid.cell);
    const int face_depth_cells = num_cells_in_len(parameters.face_depth, band_grid.cell);

    std::vector<detail::WallFaceSample> samples;
    for (int row = 0; row + cells_per_strip <= row_count; row += cells_per_strip) {
        const Strip strip{direction, row, cells_per_strip, walk_cell_count};
        const int outermost_cell = find_outermost_room_cell(space, room, band_grid, strip);
        if (outermost_cell < 0) continue;
        detail::WallFaceSample sample;
        sample.from = along_start + row * band_grid.cell;
        sample.to = sample.from + cells_per_strip * band_grid.cell;
        const int wall_cell =
            find_wall_cell(band_grid, strip, outermost_cell, max_reach_cells, max_gap_cells);
        if (wall_cell >= 0) sample.face = find_face(band_grid, strip, wall_cell, face_depth_cells);
        samples.push_back(sample);
    }
    return samples;
}

// neighbouring strips that agree on one position
struct Segment {
    double from = 0.0, to = 0.0;
    std::vector<double> faces;
};

double segment_length(const Segment& segment) { return segment.to - segment.from; }

/// Takes samples of wall faces and splits them into segments and merge them
/// into a single segment if they are close enough.
std::vector<Segment> split_into_segments(const std::vector<detail::WallFaceSample>& samples,
                                         double segment_jump) {
    std::vector<Segment> segments;
    for (const detail::WallFaceSample& sample : samples) {
        if (!sample.face) continue;
        if (!segments.empty() &&
            std::abs(*sample.face - median(segments.back().faces)) <= segment_jump) {
            segments.back().faces.push_back(*sample.face);
            segments.back().to = sample.to;
        } else {
            segments.push_back({sample.from, sample.to, {*sample.face}});
        }
    }
    return segments;
}

/// Merges close segments by extending them to cover the gap, so they are treated as a single
/// segment.
void merge_close_segments(std::vector<Segment>& segments, double merge_distance) {
    std::vector<Segment> merged;
    for (Segment& segment : segments) {
        if (!merged.empty() &&
            std::abs(median(merged.back().faces) - median(segment.faces)) <= merge_distance) {
            Segment& previous = merged.back();
            previous.faces.insert(previous.faces.end(), segment.faces.begin(), segment.faces.end());
            previous.from = std::min(previous.from, segment.from);
            previous.to = std::max(previous.to, segment.to);
        } else {
            merged.push_back(std::move(segment));
        }
    }
    segments = std::move(merged);
}

// a short segment lying a little inward of a neighbour is furniture against the wall: the neighbour
// grows over it and the strips of the dent do not vote
// repeated until nothing changes, because removing one dent can expose the next
void bridge_dents(std::vector<Segment>& segments, int outward, const OutlineParams& parameters) {
    bool bridged_one = true;
    while (bridged_one) {
        bridged_one = false;
        const int segment_count = static_cast<int>(segments.size());
        for (int dent_index = 0; dent_index < segment_count && !bridged_one; ++dent_index) {
            const Segment& dent = segments[static_cast<std::size_t>(dent_index)];
            if (segment_length(dent) > parameters.dent_length + kEpsilon) continue;

            // closer to the real-possible value
            const double dent_face = median(dent.faces);
            int host_index = -1;
            for (const int neighbour_index : {dent_index - 1, dent_index + 1}) {
                if (neighbour_index < 0 || neighbour_index >= segment_count) continue;
                const Segment& neighbour = segments[static_cast<std::size_t>(neighbour_index)];
                const double outward_offset = (median(neighbour.faces) - dent_face) * outward;
                // the dent is already part of the allignment || the dent is too deep
                if (outward_offset <= parameters.merge_distance ||
                    outward_offset > parameters.dent_depth + kEpsilon) {
                    continue;
                }
                if (host_index < 0 ||
                    segment_length(neighbour) >
                        segment_length(segments[static_cast<std::size_t>(host_index)])) {
                    host_index = neighbour_index;
                }
            }
            if (host_index < 0) continue;

            Segment& host = segments[static_cast<std::size_t>(host_index)];
            host.from = std::min(host.from, dent.from);
            host.to = std::max(host.to, dent.to);
            segments.erase(segments.begin() + dent_index);
            bridged_one = true;
        }
    }
}

// too few strips agree to call it a wall
void drop_short_segments(std::vector<Segment>& segments, int min_strips_per_segment) {
    segments.erase(std::remove_if(segments.begin(), segments.end(),
                                  [&](const Segment& segment) {
                                      return static_cast<int>(segment.faces.size()) <
                                             min_strips_per_segment;
                                  }),
                   segments.end());
}

// the end of side that heads towards heading: -1 from, +1 to
double end_towards(const OutlineSide& side, int heading) {
    return heading < 0 ? side.from : side.to;
}

/// Where the end of side heading towards end_heading meets a perpendicular wall: the face of the
/// nearest wall that faces out that way, when each of the two reaches the other's face (short of
/// it by up to corner_max_extension, past it by up to corner_overshoot). The end itself when no
/// wall fits, so the end stays where it was seen.
double find_corner(const OutlineSide& side, int end_heading, const std::vector<OutlineSide>& sides,
                   const OutlineParams& parameters) {
    // signed gap along the way an end heads: positive = stretch, negative = trim back
    const auto fits = [&](double gap) {
        return gap >= -parameters.corner_overshoot - kEpsilon &&
               gap <= parameters.corner_max_extension + kEpsilon;
    };
    const double end = end_towards(side, end_heading);
    double corner = end, nearest_gap = std::numeric_limits<double>::max();
    for (const OutlineSide& other : sides) {
        if (other.direction.axis == side.direction.axis || other.direction.outward != end_heading) {
            continue;
        }
        const double other_gap =
            (side.face - end_towards(other, side.direction.outward)) * side.direction.outward;
        const double gap = (other.face - end) * end_heading;
        if (!fits(other_gap) || !fits(gap) || std::abs(gap) >= nearest_gap) continue;
        nearest_gap = std::abs(gap);
        corner = other.face;
    }
    return corner;
}

}  // namespace

std::vector<OutlineSide> detail::find_outline_sides(const std::vector<detail::WallFaceSample>& samples,
                                            Direction direction, const OutlineParams& parameters) {
    std::vector<Segment> segments = split_into_segments(samples, parameters.segment_jump);
    drop_short_segments(segments, parameters.min_strips_per_segment);
    merge_close_segments(segments, parameters.merge_distance);
    bridge_dents(segments, direction.outward, parameters);
    // a bridge can bring two outer segments next to each other
    merge_close_segments(segments, parameters.merge_distance);

    std::vector<OutlineSide> sides;
    sides.reserve(segments.size());
    for (const Segment& segment : segments) {
        sides.push_back({direction, median(segment.faces), segment.from, segment.to});
    }
    return sides;
}

void detail::join_outline_corners(std::vector<OutlineSide>& sides, const OutlineParams& parameters) {
    // checks read the sides as they were, so a side already moved never changes the next checks
    const std::vector<OutlineSide> sides_before = sides;
    for (std::size_t i = 0; i < sides.size(); ++i) {
        const double from = find_corner(sides_before[i], -1, sides_before, parameters);
        const double to = find_corner(sides_before[i], 1, sides_before, parameters);
        sides[i].from = std::min(from, to);
        sides[i].to = std::max(from, to);
    }
}

namespace {

std::vector<OutlineSide> find_room_outline(const FreeSpace& space, std::uint32_t room,
                                           const Grid& band_grid, const OutlineParams& parameters) {
    std::vector<OutlineSide> outline;
    for (const Direction direction : kWaysOut) {
        const std::vector<OutlineSide> sides =
            detail::find_outline_sides(sample_wall_faces(space, room, band_grid, direction, parameters),
                               direction, parameters);
        outline.insert(outline.end(), sides.begin(), sides.end());
    }
    detail::join_outline_corners(outline, parameters);
    return outline;
}

}  // namespace

std::vector<std::vector<OutlineSide>> find_room_outlines(const FreeSpace& space, const Grid& grid,
                                                         const std::vector<LidarPoint>& aligned,
                                                         const Plane& floor, double band_top_metres,
                                                         const OutlineParams& parameters) {
    // same points and cell as grid, so the same cells: room_of_cell indexes both
    const Grid band_grid =
        build_grid(aligned, floor, parameters.band_lo, band_top_metres, grid.cell);
    std::vector<std::vector<OutlineSide>> outlines(space.room_count);
    if (band_grid.column.size() != space.room_of_cell.size()) return outlines;
    for (std::uint32_t room = 1; room <= space.room_count; ++room) {
        outlines[room - 1] = find_room_outline(space, room, band_grid, parameters);
    }
    return outlines;
}

}  // namespace vcortex
