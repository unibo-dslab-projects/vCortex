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
};

struct ObjectParams {
    double cell = 0.05;          // the map_builder voxel, so one voxel column per cell
    int cell_min_points = 1;     // points in a cell to call it occupied
    double close_radius = 0.10;  // gaps up to twice this fill in
    double split_radius = 0.10;  // parts thinner than twice this stop gluing objects together
    double height_step = 0.0;    // 0 = off; else a top jump this big between cells splits a piece
    double min_box_side = 0.20;  // a thinner part of an object joins the nearest box of it
    int min_piece_cells = 8;     // a smaller region of the height split joins a neighbour
    int cluster_min_points = 20;
};

struct WallParams {
    double collinear_tolerance = 0.10;
    double min_gap = 0.30;           // shorter gaps are closed
    double door_max = 1.2;           // gaps up to this stay openings
    double corner_gap = 0.5;         // wall ends this close to a perpendicular wall are joined
    double thickness = 0.10;         // used when only one face was seen
    double object_clearance = 0.10;  // band points closer than this to a wall face are wall
};

struct RoomParams {
    double min_area = 1.0;              // square metres; smaller fills are pockets
    double wall_touch_distance = 0.15;  // wall faces this close to the outline belong to the room
};

struct FreeSpaceParams {
    double floor_band = 0.08;  // metres around the floor plane; the floor layer is this thick
    double low_lo = 0.10;      // wall and glass bases, seen under desks by a low lidar
    double low_hi = 0.30;
    std::uint16_t min_points = 2;  // one voxel alone is noise
    double tall_from = 0.90;       // metres; desks stay below, walls and wardrobes reach it
    double door_radius = 0.50;     // gaps between tall cells narrower than twice this are doors
};

struct OutlineParams {
    double band_lo = 0.10;       // metres above the floor; wall and glass bases show under desks
    double strip = 0.25;         // rows of cells walked together; they fill each other's holes
    double max_reach = 1.5;      // walk past the free space at most this far
    double max_gap = 0.30;       // this much empty floor ends the walk: what lies past is outside
    double face_depth = 0.10;    // band points this deep up to the last occupied cell give the face
    double segment_jump = 0.15;  // a face this far from its segment starts a new segment
    int min_strips_per_segment = 2;  // shorter segments are a leg or a bag, not a wall
    double merge_distance = 0.10;    // neighbour segments this close are one line
    double dent_depth = 0.50;        // segments at most this far in, and at most dent_length long,
    double dent_length = 2.0;        //   are furniture against the wall
    // desks along the walls hide metres of wall: a 1.5 m reach left a furnished office open
    double corner_max_extension = 8.0;
    double corner_overshoot = 0.5;  // a side may poke this far past the corner; trimmed back
};

struct ModelSettings {
    double wall_height = 2.70;  // metres, replaced by the user's value
    HeightSource wall_height_source = HeightSource::Default;
    int outlier_mean_k = 20;
    double outlier_stddev_mul = 2.0;
    CandidateParams candidates;
    ObjectParams objects;
    WallParams walls;
    RoomParams rooms;
    FreeSpaceParams free_space;
    OutlineParams outline;
};

inline constexpr double kWallHeightMin = 1.5;
inline constexpr double kWallHeightMax = 6.0;

/// validate the models settings
std::optional<std::string> validate(const ModelSettings& s);

}  // namespace vcortex
