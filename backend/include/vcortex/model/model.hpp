#pragma once

#include <cstddef>
#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/model/floor/floor.hpp"

namespace vcortex {

/// Diagnostics of one model run: what each step found, to judge a map without opening it.
struct ModelReport {
    std::size_t points_in = 0, after_outliers = 0;
    Plane floor;            // map frame
    double tilt_deg = 0.0;  // floor slope from horizontal; a few degrees means a tilted sensor
    double yaw_deg = 0.0;   // turn that squares the map; walls run along x or y after it
    // poses used; 0: no inside/outside, so the outermost overlapping parallel candidate is wall
    std::size_t poses = 0;
    std::size_t candidates = 0, walls = 0, rooms = 0, objects = 0;
    // floor reached from the poses, under desks too; a jump between runs means a hole
    double free_space_area = 0.0;
    std::size_t free_space_rooms = 0;  // rooms the poses walked into
    std::size_t wall_faces_found = 0;  // all 4 directions; low means the shell is barely seen
    // 0: not exactly one room, or none seen; fewer than 4: a direction found no wall face
    std::size_t outline_sides = 0;
    double shell_top_seen = 0.0;  // what walls.min_top_ratio is a share of; 0: no shell
    double duration_ms = 0.0;
};

/// Everything one run produces.
struct ModelResult {
    Building building;
    std::vector<LidarPoint> cleaned;  // after drop_outliers, for the ply connector
    ModelReport report;
};

/// Turns one map and the robot's trajectory into a building: floor, walls, rooms and objects.
// pure, no i/o: live, offline and tests run the same code
// building returned in the source map frame, not aligned, so it overlays the raw cloud as is
ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings);

}  // namespace vcortex
