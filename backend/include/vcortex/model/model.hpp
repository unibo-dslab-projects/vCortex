#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

struct ModelReport {
    std::size_t points_in = 0, after_transient = 0, after_outliers = 0;
    bool transient_skipped = false;  // the input had no hits
    Plane floor;
    double tilt_deg = 0.0;
    double yaw_deg = 0.0;
    // Poses the model used. 0 means split_walls_objects could not tell inside from outside and
    // fell back to "the outermost of overlapping parallel candidates is the wall".
    std::size_t poses = 0;
    std::size_t candidates = 0, walls = 0, rooms = 0, objects = 0;
    double duration_ms = 0.0;
};

struct ModelResult {
    Building building;
    std::vector<Point> cleaned;  // after drop_transient and drop_outliers, for the PLY connector
    // Copied from the Snapshot, so the PLY connector writes a hits or intensity column only when
    // it holds measured values and not the 0 placeholders.
    bool has_hits = false;
    bool has_intensity = false;
    ModelReport report;
};

// Pure function, no I/O: live mode, offline mode and tests run the same code on the same input.
// The building comes back in the source's map frame, not the aligned one, so it overlays the raw
// cloud in the browser without a transform.
ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings);

}  // namespace vcortex
