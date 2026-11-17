#include "vcortex/domain/settings.hpp"

#include <sstream>

namespace vcortex {

namespace {

std::string problem(const char* key, const char* rule, double got) {
    std::ostringstream out;
    out << key << " must be " << rule << " (got " << got << ")";
    return out.str();
}

}  // namespace

// !(ok) form: nan fails every comparison, so it is refused too
std::optional<std::string> validate(const ModelSettings& s) {
    if (!(s.wall_height > kWallHeightMin && s.wall_height < kWallHeightMax)) {
        std::ostringstream rule;
        rule << "in (" << kWallHeightMin << ", " << kWallHeightMax << ")";
        return problem("wall_height", rule.str().c_str(), s.wall_height);
    }
    if (s.outlier_mean_k < 1) return problem("outliers.mean_k", ">= 1", s.outlier_mean_k);
    if (!(s.outlier_stddev_mul > 0)) {
        return problem("outliers.stddev_mul", "> 0", s.outlier_stddev_mul);
    }

    const CandidateParams& c = s.candidates;
    if (!(c.cell > 0)) return problem("candidates.cell", "> 0", c.cell);
    if (!(c.band_lo >= 0)) return problem("candidates.band_lo", ">= 0", c.band_lo);
    if (!(c.band_top_margin >= 0)) {
        return problem("candidates.band_top_margin", ">= 0", c.band_top_margin);
    }
    // empty band leaves the grid blank: no walls reported, no error
    if (!(c.band_lo < s.wall_height - c.band_top_margin)) {
        return problem("candidates.band_lo", "below wall_height - candidates.band_top_margin",
                       c.band_lo);
    }
    if (c.min_column < 1) return problem("candidates.min_column", ">= 1", c.min_column);
    if (!(c.max_gap >= 0)) return problem("candidates.max_gap", ">= 0", c.max_gap);
    if (!(c.min_length > 0)) return problem("candidates.min_length", "> 0", c.min_length);
    if (!(c.merge_distance >= 0)) {
        return problem("candidates.merge_distance", ">= 0", c.merge_distance);
    }

    const WallParams& w = s.walls;
    if (!(w.span_margin >= 0)) return problem("walls.span_margin", ">= 0", w.span_margin);
    if (!(w.both_sides_range > 0)) {
        return problem("walls.both_sides_range", "> 0", w.both_sides_range);
    }
    if (!(w.max_depth >= 0)) return problem("walls.max_depth", ">= 0", w.max_depth);
    if (!(w.overlap_widen >= 0)) return problem("walls.overlap_widen", ">= 0", w.overlap_widen);
    if (!(w.collinear_tolerance >= 0)) {
        return problem("walls.collinear_tolerance", ">= 0", w.collinear_tolerance);
    }
    if (!(w.min_gap >= 0)) return problem("walls.min_gap", ">= 0", w.min_gap);
    // else no gap is a door: shorter ones close, longer ones are not doors
    if (!(w.door_max >= w.min_gap)) {
        return problem("walls.door_max", ">= walls.min_gap", w.door_max);
    }
    if (!(w.corner_gap >= 0)) return problem("walls.corner_gap", ">= 0", w.corner_gap);
    if (!(w.thickness > 0)) return problem("walls.thickness", "> 0", w.thickness);
    if (!(w.object_clearance >= 0)) {
        return problem("walls.object_clearance", ">= 0", w.object_clearance);
    }
    if (!(w.min_top_ratio >= 0 && w.min_top_ratio <= 1)) {
        return problem("walls.min_top_ratio", "in [0, 1]", w.min_top_ratio);
    }

    const ObjectParams& ob = s.objects;
    if (!(ob.cell > 0)) return problem("objects.cell", "> 0", ob.cell);
    if (ob.cell_min_points < 1) {
        return problem("objects.cell_min_points", ">= 1", ob.cell_min_points);
    }
    if (!(ob.close_radius >= 0)) return problem("objects.close_radius", ">= 0", ob.close_radius);
    if (!(ob.split_radius >= 0)) return problem("objects.split_radius", ">= 0", ob.split_radius);
    if (!(ob.height_step >= 0)) return problem("objects.height_step", ">= 0", ob.height_step);
    if (!(ob.min_step >= 0)) return problem("objects.min_step", ">= 0", ob.min_step);
    if (ob.min_piece_cells < 1) {
        return problem("objects.min_piece_cells", ">= 1", ob.min_piece_cells);
    }
    if (ob.cluster_min_points < 1) {
        return problem("objects.cluster_min_points", ">= 1", ob.cluster_min_points);
    }

    const RoomParams& r = s.rooms;
    if (!(r.min_area >= 0)) return problem("rooms.min_area", ">= 0", r.min_area);
    if (!(r.wall_touch_distance >= 0)) {
        return problem("rooms.wall_touch_distance", ">= 0", r.wall_touch_distance);
    }

    const FreeSpaceParams& free_space = s.free_space;
    if (!(free_space.floor_band > 0)) {
        return problem("free_space.floor_band", "> 0", free_space.floor_band);
    }
    if (!(free_space.low_lo >= free_space.floor_band)) {
        return problem("free_space.low_lo", ">= free_space.floor_band", free_space.low_lo);
    }
    if (!(free_space.low_hi > free_space.low_lo)) {
        return problem("free_space.low_hi", "> free_space.low_lo", free_space.low_hi);
    }
    if (free_space.min_points < 1) {
        return problem("free_space.min_points", ">= 1", free_space.min_points);
    }
    if (!(free_space.tall_from >= free_space.low_hi)) {
        return problem("free_space.tall_from", ">= free_space.low_hi", free_space.tall_from);
    }
    if (!(free_space.door_radius > 0)) {
        return problem("free_space.door_radius", "> 0", free_space.door_radius);
    }

    const OutlineParams& o = s.outline;
    if (!(o.band_lo >= 0 && o.band_lo < s.wall_height - c.band_top_margin)) {
        return problem("outline.band_lo", ">= 0 and below wall_height - candidates.band_top_margin",
                       o.band_lo);
    }
    // a strip narrower than a cell holds no whole cell
    // one missing cell must not end the walk
    if (!(o.max_gap >= c.cell)) return problem("outline.max_gap", ">= candidates.cell", o.max_gap);
    // else every walk ends at the reach, never at a gap
    if (!(o.max_reach > o.max_gap)) {
        return problem("outline.max_reach", "> outline.max_gap", o.max_reach);
    }
    // the window then covers the whole last occupied cell, so it is never empty
    if (!(o.segment_jump > 0)) return problem("outline.segment_jump", "> 0", o.segment_jump);
    if (!(o.strip >= c.cell)) return problem("outline.strip", ">= candidates.cell", o.strip);
    if (!(o.face_depth >= c.cell)) {
        return problem("outline.face_depth", ">= candidates.cell", o.face_depth);
    }
    if (o.min_strips_per_segment < 1) {
        return problem("outline.min_strips_per_segment", ">= 1", o.min_strips_per_segment);
    }
    // else merging undoes every split made by segment_jump
    if (!(o.merge_distance >= 0 && o.merge_distance < o.segment_jump)) {
        return problem("outline.merge_distance", ">= 0 and < outline.segment_jump", o.merge_distance);
    }
    // a dent must be deeper than a merge, or it would already be one line
    if (!(o.dent_depth > o.merge_distance)) {
        return problem("outline.dent_depth", "> outline.merge_distance", o.dent_depth);
    }
    if (!(o.dent_length > 0)) return problem("outline.dent_length", "> 0", o.dent_length);
    if (!(o.corner_max_extension >= 0)) {
        return problem("outline.corner_max_extension", ">= 0", o.corner_max_extension);
    }
    if (!(o.corner_overshoot >= 0)) {
        return problem("outline.corner_overshoot", ">= 0", o.corner_overshoot);
    }

    return std::nullopt;
}

}  // namespace vcortex
