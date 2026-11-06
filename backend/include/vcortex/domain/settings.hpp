#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace vcortex {

// start values for map_builder's 5 cm voxel maps, untuned; config/model.json overrides

enum class HeightSource { Default, User };

struct CandidateParams {
    double band_lo = 0.30;          // metres above the floor; cuts floor noise and skirting
    double band_top_margin = 0.10;  // band top = wall_height - this
    double cell = 0.05;             // grid cell size, metres
    std::uint16_t min_column = 2;   // voxels per cell to call it occupied (map_builder maps)
    double max_gap = 0.30;          // gaps bridged inside one run
    double min_length = 1.0;        // shorter runs are dropped
    double merge_distance = 0.10;   // parallel runs closer than this become one
};

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

struct RoomParams {
    double min_area = 1.0;               // square metres; smaller fills are pockets
    double wall_touch_distance = 0.15;   // wall faces this close to the outline belong to the room
};

struct RoomSpaceParams {
    double floor_band = 0.08;      // metres around the floor plane; the floor layer is this thick
    double low_lo = 0.10;          // wall and glass bases, seen under desks by a low lidar
    double low_hi = 0.30;
    std::uint16_t min_points = 2;  // one voxel alone is noise
    double coin_radius = 0.15;     // gaps narrower than twice this keep the coin in
};

struct OutlineParams {
    double strip = 0.25;       // strip width along the wall; one face per strip
    double max_reach = 1.5;    // walk past the room space at most this far
    double max_gap = 0.30;     // this much empty floor ends the walk: what lies past is outside
    double face_depth = 0.10;  // points this deep in the last occupied cell give the face
    double run_jump = 0.15;    // a face this far from its run starts a new run
    int min_strips_per_run = 2;          // shorter runs are a leg or a bag, not a wall
    double merge_distance = 0.10;        // neighbour runs this close are one line
    double dent_depth = 0.50;   // runs at most this far in, and at most dent_length long,
    double dent_length = 2.0;   //   are furniture against the wall
    double corner_reach = 1.5;  // corners hold furniture as deep as max_reach
};

struct ModelSettings {
    double wall_height = 2.70;  // metres, replaced by the user's value
    HeightSource wall_height_source = HeightSource::Default;
    int outlier_mean_k = 20;
    double outlier_stddev_mul = 2.0;
    CandidateParams candidates;
    WallParams walls;
    RoomParams rooms;
    RoomSpaceParams room_space;
    OutlineParams outline;
};

// below 1.5 m the band is too thin to tell walls from furniture; above 6 m it is not one
// storey; both ends excluded
inline constexpr double kWallHeightMin = 1.5;
inline constexpr double kWallHeightMax = 6.0;

// guards config/model.json and PUT /api/settings, so bad values fail at entry, not mid-run
// returns the first problem by json key (e.g. "candidates.cell"), or nothing
std::optional<std::string> validate(const ModelSettings& s);

}  // namespace vcortex
