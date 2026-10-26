#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

// Stage G (step 11): run A -> F and assemble the building in the source's map frame.

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
    std::vector<Point> cleaned;  // after stage A, for the PLY connector
    ModelReport report;
};

// Pure function: same input, same output, no I/O.
ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings);

}  // namespace vcortex
