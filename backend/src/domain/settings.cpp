#include "vcortex/domain/settings.hpp"

#include <initializer_list>
#include <limits>
#include <sstream>

namespace vcortex {

namespace {

constexpr double kNoMax = std::numeric_limits<double>::infinity();
// smallest gap between two limits that still means "more than" (1 mm for a length)
constexpr double kSmallestStep = 0.001;
// growing a mask by r cells costs about r^2 reads per cell: 20 keeps a room near 10^8
constexpr double kMaxRadiusCells = 20;

/// A setting and the range its value must lie in, both ends included.
struct Bound {
    template <class Number>
    Bound(const char* setting_key, Number setting_value, double lowest, double highest)
        : key(setting_key),
          value(static_cast<double>(setting_value)),
          minimum(lowest),
          maximum(highest) {}

    const char* key;
    double value, minimum, maximum;
};

// nan and inf fail the comparison, so they are refused too
std::optional<std::string> find_violation(std::initializer_list<Bound> bounds) {
    for (const Bound& bound : bounds) {
        if (bound.minimum <= bound.value && bound.value <= bound.maximum) continue;

        std::ostringstream message;
        message << bound.key << " must be in [" << bound.minimum << ", " << bound.maximum
                << "] (got " << bound.value << ")";
        return message.str();
    }
    return std::nullopt;
}

}  // namespace

// a limit may use an earlier setting, so a row sits below the rows it reads
std::optional<std::string> validate(const ModelSettings& s) {
    const CandidateParams& c = s.candidates;
    const ObjectParams& ob = s.objects;
    const WallParams& w = s.walls;
    const RoomParams& r = s.rooms;
    const FreeSpaceParams& f = s.free_space;
    const OutlineParams& o = s.outline;
    const double wall_height = s.wall_height;
    // an empty band leaves the grid blank: no walls reported, no error
    const double band_top = wall_height - c.band_top_margin - kSmallestStep;

    return find_violation({
        {"wall_height", wall_height, kWallHeightMin, kWallHeightMax},
        {"outliers.mean_k", s.outlier_mean_k, 1, 200},
        {"outliers.stddev_mul", s.outlier_stddev_mul, kSmallestStep, 10},

        {"candidates.cell", c.cell, 0.02, 0.5},
        {"candidates.band_top_margin", c.band_top_margin, 0, wall_height},
        {"candidates.band_lo", c.band_lo, 0, band_top},
        {"candidates.min_column", c.min_column, 1, kNoMax},

        {"objects.cell", ob.cell, 0.02, 0.5},
        {"objects.cell_min_points", ob.cell_min_points, 1, kNoMax},
        {"objects.close_radius", ob.close_radius, 0, kMaxRadiusCells * ob.cell},
        {"objects.split_radius", ob.split_radius, 0, kMaxRadiusCells * ob.cell},
        {"objects.height_step", ob.height_step, 0, wall_height},
        {"objects.min_box_side", ob.min_box_side, 0, 10},
        {"objects.min_piece_cells", ob.min_piece_cells, 1, kNoMax},
        {"objects.cluster_min_points", ob.cluster_min_points, 1, kNoMax},

        {"walls.collinear_tolerance", w.collinear_tolerance, 0, 1},
        {"walls.min_gap", w.min_gap, 0, 5},
        // below min_gap every gap closes, so no gap would be a door
        {"walls.door_max", w.door_max, w.min_gap, 5},
        {"walls.corner_gap", w.corner_gap, 0, 5},
        {"walls.thickness", w.thickness, kSmallestStep, 1},
        {"walls.object_clearance", w.object_clearance, 0, 1},

        {"rooms.min_area", r.min_area, 0, 1000},
        {"rooms.wall_touch_distance", r.wall_touch_distance, 0, 2},

        {"free_space.floor_band", f.floor_band, kSmallestStep, wall_height},
        {"free_space.low_lo", f.low_lo, f.floor_band, wall_height},
        {"free_space.low_hi", f.low_hi, f.low_lo + kSmallestStep, wall_height},
        {"free_space.min_points", f.min_points, 1, kNoMax},
        {"free_space.tall_from", f.tall_from, f.low_hi, wall_height},
        {"free_space.door_radius", f.door_radius, kSmallestStep, kMaxRadiusCells * c.cell},

        {"outline.band_lo", o.band_lo, 0, band_top},
        // a strip narrower than a cell holds no whole cell
        {"outline.strip", o.strip, c.cell, 2},
        {"outline.max_gap", o.max_gap, c.cell, 5},
        // at or below max_gap every walk ends at a gap, never at the reach
        {"outline.max_reach", o.max_reach, o.max_gap + kSmallestStep, 10},
        {"outline.face_depth", o.face_depth, c.cell, 1},
        {"outline.segment_jump", o.segment_jump, kSmallestStep, 1},
        {"outline.min_strips_per_segment", o.min_strips_per_segment, 1, kNoMax},
        // at or above segment_jump, merging undoes every split
        {"outline.merge_distance", o.merge_distance, 0, o.segment_jump - kSmallestStep},
        // a dent must be deeper than a merge, or it would already be one line
        {"outline.dent_depth", o.dent_depth, o.merge_distance + kSmallestStep, 10},
        {"outline.dent_length", o.dent_length, kSmallestStep, 20},
        {"outline.corner_max_extension", o.corner_max_extension, 0, 20},
        {"outline.corner_overshoot", o.corner_overshoot, 0, 2},
    });
}

}  // namespace vcortex
