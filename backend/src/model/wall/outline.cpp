#include "vcortex/model/wall/outline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "vcortex/model/median.hpp"

namespace vcortex {

namespace {

constexpr double kEpsilon = 1e-9;

// neighbouring strips that agree on one position
struct Segment {
    double from = 0.0, to = 0.0;
    std::vector<double> faces;
};

double segment_length(const Segment& segment) { return segment.to - segment.from; }

// strips without a face neither split nor join: a door still belongs to the same wall line
// the reference is the segment median, so one odd strip does not drag it
std::vector<Segment> split_into_segments(const std::vector<WallFaceSample>& samples,
                                         double segment_jump) {
    std::vector<Segment> segments;
    for (const WallFaceSample& sample : samples) {
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

/// Merges close segments by extending them to cover the gap, so they are treated as a single segment.
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
                    segment_length(neighbour) > segment_length(segments[static_cast<std::size_t>(host_index)])) {
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

}  // namespace

std::vector<OutlineSide> find_outline_sides(const std::vector<WallFaceSample>& samples,
                                            Direction direction, const OutlineParams& parameters) {
    std::vector<Segment> segments = split_into_segments(samples, parameters.segment_jump);

    segments.erase(std::remove_if(segments.begin(), segments.end(),
                              [&](const Segment& segment) {
                                  return static_cast<int>(segment.faces.size()) <
                                         parameters.min_strips_per_segment;
                              }),
               segments.end());

    merge_close_segments(segments, parameters.merge_distance);
    bridge_dents(segments, direction.outward, parameters);
    // a bridge can bring two outer segments next to each other
    merge_close_segments(segments, parameters.merge_distance);

    std::vector<OutlineSide> sides;
    sides.reserve(segments.size());
    for (const Segment& segment : segments) {
        sides.push_back({direction, median(segment.faces), segment.from, segment.to, segment.faces.size()});
    }
    return sides;
}

void join_outline_corners(std::vector<OutlineSide>& sides, const OutlineParams& parameters) {
    // read from a copy, so the order of the sides does not change the result
    const std::vector<OutlineSide> sides_before = sides;
    // signed gap along the way an end points: positive = stretch, negative = trim back
    const auto fits = [&](double gap) {
        return gap >= -parameters.corner_overshoot - kEpsilon &&
               gap <= parameters.corner_max_extension + kEpsilon;
    };
    for (OutlineSide& side : sides) {
        // -1: the from end points to smaller coordinates, +1: the to end to larger ones
        for (const int end_heading : {-1, 1}) {
            double& end = end_heading < 0 ? side.from : side.to;
            double nearest_distance = std::numeric_limits<double>::max();
            double nearest_face = end;
            for (const OutlineSide& other : sides_before) {
                if (other.direction.axis == side.direction.axis) continue;
                // a room corner: this end heads the way the other side faces out, and the other
                // side's end that heads the way this side faces out reaches towards this face
                if (other.direction.outward != end_heading) continue;
                const double other_end = side.direction.outward < 0 ? other.from : other.to;
                if (!fits((side.face - other_end) * side.direction.outward)) continue;
                const double gap = (other.face - end) * end_heading;
                if (!fits(gap)) continue;
                if (std::abs(gap) < nearest_distance) {
                    nearest_distance = std::abs(gap);
                    nearest_face = other.face;
                }
            }
            end = nearest_face;
        }
        if (side.from > side.to) std::swap(side.from, side.to);
    }
}

void measure_outline_coverage(std::vector<OutlineSide>& sides,
                              const std::vector<LidarPoint>& aligned_points,
                              const Plane& aligned_floor, double band_bottom_metres,
                              double band_top_metres, double cell_size_metres,
                              double coverage_tolerance_metres) {
    for (OutlineSide& side : sides) {
        const bool is_x_axis = side.direction.axis == Axis::X;
        const double side_length_metres = std::max(0.0, side.to - side.from);
        const auto step_count = static_cast<std::size_t>(
            std::ceil(side_length_metres / cell_size_metres - kEpsilon));
        std::vector<bool> step_has_point(step_count, false);

        for (const LidarPoint& point : aligned_points) {
            const double height_metres = height_above(aligned_floor, point);
            if (height_metres < band_bottom_metres || height_metres >= band_top_metres) continue;

            const double normal_coordinate = is_x_axis ? point.position.x() : point.position.y();
            const double along_coordinate = is_x_axis ? point.position.y() : point.position.x();
            if (std::abs(normal_coordinate - side.face) > coverage_tolerance_metres + kEpsilon) {
                continue;
            }
            if (along_coordinate < side.from || along_coordinate >= side.to) continue;

            const auto step_index =
                static_cast<std::size_t>((along_coordinate - side.from) / cell_size_metres);
            if (step_index < step_count) step_has_point[step_index] = true;
        }
        side.seen_cell_count = static_cast<std::size_t>(
            std::count(step_has_point.begin(), step_has_point.end(), true));
    }
}

}  // namespace vcortex
