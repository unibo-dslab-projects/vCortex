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
    std::size_t points_in = 0, after_outliers = 0;
    Plane floor;
    double tilt_deg = 0.0;
    double yaw_deg = 0.0;
    // poses used; 0: no inside/outside, so the outermost overlapping parallel candidate is wall
    std::size_t poses = 0;
    std::size_t candidates = 0, walls = 0, rooms = 0, objects = 0;
    // floor a coin of room_space.coin_radius reaches from the poses; a jump between runs means a hole
    double room_space_area = 0.0;
    bool room_space_leaked = false;
    std::size_t wall_faces_found = 0;  // all 4 directions; low means the shell is barely seen
    // 0: room space leaked or not seen; fewer than 4: a direction found no wall face
    std::size_t outline_sides = 0;
    double shell_top_seen = 0.0;  // what walls.min_top_ratio is a share of; 0: no shell
    double duration_ms = 0.0;
};

struct ModelResult {
    Building building;
    std::vector<Point> cleaned;  // after drop_outliers, for the ply connector
    ModelReport report;
};

// pure, no i/o: live, offline and tests run the same code
// building returned in the source map frame, not aligned, so it overlays the raw cloud as is
ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings);

}  // namespace vcortex
