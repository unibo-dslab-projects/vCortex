#pragma once

#include <cstdint>

namespace vcortex {

// Every value below is a start value from the plan. config/model.json overrides them.

enum class HeightSource { Default, User };

// Stage D (step 8).
struct CandidateParams {
    double band_lo = 0.30;          // metres above the floor; cuts floor noise and skirting
    double band_top_margin = 0.10;  // band top = wall_height - this
    double cell = 0.05;             // grid cell size, metres
    std::uint16_t min_column = 2;   // voxels per cell to call it occupied (map_builder maps)
    double max_gap = 0.30;          // gaps bridged inside one run
    double min_length = 1.0;        // shorter runs are dropped
    double merge_distance = 0.10;   // parallel runs closer than this become one
};

// Stage E (step 9).
struct WallParams {
    double span_margin = 0.5;        // poses within the span +- this count for the interior side
    double both_sides_range = 5.0;   // poses farther than this are ignored
    double max_depth = 1.5;          // wardrobe rule: how far behind a candidate to look
    double overlap_widen = 1.0;      // wardrobe rule: span widening on each end
    double collinear_tolerance = 0.10;
    double min_gap = 0.30;           // shorter gaps are closed
    double door_max = 1.2;           // gaps up to this stay openings
    double corner_gap = 0.5;         // wall ends this close to a perpendicular wall are joined
    double thickness = 0.10;         // used when only one face was seen
    double object_clearance = 0.10;  // band points closer than this to a wall face are wall
    double cluster_tolerance = 0.10;
    int cluster_min_points = 20;
    double against_wall_distance = 0.15;
};

// Stage F (step 10).
struct RoomParams {
    double min_area = 1.0;               // square metres; smaller fills are pockets
    double wall_touch_distance = 0.15;   // wall faces this close to the outline belong to the room
};

struct ModelSettings {
    double wall_height = 2.70;  // metres, replaced by the user's value
    HeightSource wall_height_source = HeightSource::Default;
    std::uint16_t min_scans = 5;  // stage A; the UI allows 1..64 (map_builder's hits_report_cap)
    int outlier_mean_k = 20;      // stage A
    double outlier_stddev_mul = 2.0;
    CandidateParams candidates;
    WallParams walls;
    RoomParams rooms;
};

}  // namespace vcortex
