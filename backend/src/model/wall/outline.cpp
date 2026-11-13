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
struct Run {
    double from = 0.0, to = 0.0;
    std::vector<double> faces;
};

double run_length(const Run& run) { return run.to - run.from; }

// strips without a face neither split nor join: a door still belongs to the same wall line
// the reference is the run median, so one odd strip does not drag it
std::vector<Run> merge_walls_and_split_into_runs(const std::vector<WallFaceSample>& samples, double run_jump) {
    std::vector<Run> runs;
    for (const WallFaceSample& sample : samples) {
        if (!sample.face) continue;
        if (!runs.empty() && std::abs(*sample.face - median(runs.back().faces)) <= run_jump) {
            runs.back().faces.push_back(*sample.face);
            runs.back().to = sample.to;
        } else {
            runs.push_back({sample.from, sample.to, {*sample.face}});
        }
    }
    return runs;
}

/// Merges close runs by extending them to cover the gap, so they are treated as a single run.
void merge_close_runs(std::vector<Run>& runs, double merge_distance) {
    std::vector<Run> merged;
    for (Run& run : runs) {
        if (!merged.empty() &&
            std::abs(median(merged.back().faces) - median(run.faces)) <= merge_distance) {
            Run& previous = merged.back();
            previous.faces.insert(previous.faces.end(), run.faces.begin(), run.faces.end());
            previous.from = std::min(previous.from, run.from);
            previous.to = std::max(previous.to, run.to);
        } else {
            merged.push_back(std::move(run));
        }
    }
    runs = std::move(merged);
}

// a short run lying a little inward of a neighbour is furniture against the wall: the neighbour
// grows over it and the strips of the dent do not vote
// repeated until nothing changes, because removing one dent can expose the next
void bridge_dents(std::vector<Run>& runs, int outward, const OutlineParams& parameters) {
    bool bridged_one = true;
    while (bridged_one) {
        bridged_one = false;
        const int run_count = static_cast<int>(runs.size());
        for (int dent_index = 0; dent_index < run_count && !bridged_one; ++dent_index) {
            const Run& dent = runs[static_cast<std::size_t>(dent_index)];
            if (run_length(dent) > parameters.dent_length + kEpsilon) continue;

            // closer to the real-possible value
            const double dent_face = median(dent.faces);
            int host_index = -1;
            for (const int neighbour_index : {dent_index - 1, dent_index + 1}) {
                if (neighbour_index < 0 || neighbour_index >= run_count) continue;
                const Run& neighbour = runs[static_cast<std::size_t>(neighbour_index)];
                const double outward_offset = (median(neighbour.faces) - dent_face) * outward;
                // the dent is already part of the allignment || the dent is too deep
                if (outward_offset <= parameters.merge_distance ||
                    outward_offset > parameters.dent_depth + kEpsilon) {
                    continue;
                }
                if (host_index < 0 ||
                    run_length(neighbour) > run_length(runs[static_cast<std::size_t>(host_index)])) {
                    host_index = neighbour_index;
                }
            }
            if (host_index < 0) continue;

            Run& host = runs[static_cast<std::size_t>(host_index)];
            host.from = std::min(host.from, dent.from);
            host.to = std::max(host.to, dent.to);
            runs.erase(runs.begin() + dent_index);
            bridged_one = true;
        }
    }
}

}  // namespace

std::vector<OutlineSide> find_outline_sides(const std::vector<WallFaceSample>& samples,
                                            Direction direction, const OutlineParams& parameters) {
    std::vector<Run> runs = merge_walls_and_split_into_runs(samples, parameters.run_jump);

    runs.erase(std::remove_if(runs.begin(), runs.end(),
                              [&](const Run& run) {
                                  return static_cast<int>(run.faces.size()) <
                                         parameters.min_strips_per_run;
                              }),
               runs.end());

    merge_close_runs(runs, parameters.merge_distance);
    bridge_dents(runs, direction.outward, parameters);
    // a bridge can bring two outer runs next to each other
    merge_close_runs(runs, parameters.merge_distance);

    std::vector<OutlineSide> sides;
    sides.reserve(runs.size());
    for (const Run& run : runs) {
        sides.push_back({direction, median(run.faces), run.from, run.to, run.faces.size()});
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
