#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

struct ModelReport {
    std::size_t points_in, after_transient, after_outliers;
    bool transient_skipped;
    Plane floor;
    double tilt_deg;
    double yaw_deg;
    std::size_t candidates, walls, rooms, objects;
    double duration_ms;
};

struct ModelResult {
    Building building;
    std::vector<Point> cleaned;  // after drop_transient and drop_outliers, for the PLY connector
    ModelReport report;
};

// Pure function, no I/O: live mode, offline mode and tests run the same code on the same input.
// The building comes back in the source's map frame, not the aligned one, so it overlays the raw
// cloud in the browser without a transform.
ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings);

}  // namespace vcortex
