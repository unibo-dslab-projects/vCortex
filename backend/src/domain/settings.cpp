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

// Checks are written as !(ok) so that NaN, which fails every comparison, is refused too.
std::optional<std::string> validate(const ModelSettings& s) {
    if (!(s.wall_height > kWallHeightMin && s.wall_height < kWallHeightMax)) {
        std::ostringstream rule;
        rule << "in (" << kWallHeightMin << ", " << kWallHeightMax << ")";
        return problem("wall_height", rule.str().c_str(), s.wall_height);
    }
    if (s.min_scans < kMinScansMin || s.min_scans > kMinScansMax) {
        std::ostringstream rule;
        rule << "in " << kMinScansMin << ".." << kMinScansMax;
        return problem("min_scans", rule.str().c_str(), s.min_scans);
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
    // An empty band would leave the grid blank: the model would report no walls, not an error.
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
    // Otherwise no gap could be a door: shorter ones are closed, longer ones are not doors.
    if (!(w.door_max >= w.min_gap)) {
        return problem("walls.door_max", ">= walls.min_gap", w.door_max);
    }
    if (!(w.corner_gap >= 0)) return problem("walls.corner_gap", ">= 0", w.corner_gap);
    if (!(w.thickness > 0)) return problem("walls.thickness", "> 0", w.thickness);
    if (!(w.object_clearance >= 0)) {
        return problem("walls.object_clearance", ">= 0", w.object_clearance);
    }
    if (!(w.cluster_tolerance > 0)) {
        return problem("walls.cluster_tolerance", "> 0", w.cluster_tolerance);
    }
    if (w.cluster_min_points < 1) {
        return problem("walls.cluster_min_points", ">= 1", w.cluster_min_points);
    }
    if (!(w.against_wall_distance >= 0)) {
        return problem("walls.against_wall_distance", ">= 0", w.against_wall_distance);
    }

    const RoomParams& r = s.rooms;
    if (!(r.min_area >= 0)) return problem("rooms.min_area", ">= 0", r.min_area);
    if (!(r.wall_touch_distance >= 0)) {
        return problem("rooms.wall_touch_distance", ">= 0", r.wall_touch_distance);
    }
    return std::nullopt;
}

}  // namespace vcortex
