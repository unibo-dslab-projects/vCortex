#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

// diagnostics of one model run
struct ModelReport {
    std::size_t points_in = 0, after_transient = 0, after_outliers = 0;
    bool transient_skipped = false;  // the input had no hits
    Plane floor;
    double tilt_deg = 0.0;
    double yaw_deg = 0.0;
    // poses used; 0: no inside/outside, so the outermost overlapping parallel candidate is wall
    std::size_t poses = 0;
    std::size_t candidates = 0, walls = 0, rooms = 0, objects = 0;
    double duration_ms = 0.0;
};

struct ModelResult {
    Building building;
    std::vector<Point> cleaned;  // after drop_transient and drop_outliers, for the ply connector
    // copied from Snapshot: ply writes hits/intensity columns only when measured,
    // not 0 placeholders
    bool has_hits = false;
    bool has_intensity = false;
    ModelReport report;
};

// pure, no i/o: live, offline and tests run the same code
// building returned in the source map frame, not aligned, so it overlays the raw cloud as is
ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings);

}  // namespace vcortex
