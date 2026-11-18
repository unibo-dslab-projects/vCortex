#include "vcortex/model/model.hpp"

#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "vcortex/geometry/vectors.hpp"
#include "vcortex/model/align.hpp"
#include "vcortex/model/clean.hpp"
#include "vcortex/model/floor/free_space.hpp"
#include "vcortex/model/grid.hpp"
#include "vcortex/model/object/objects.hpp"
#include "vcortex/model/rooms.hpp"
#include "vcortex/model/wall/outline.hpp"
#include "vcortex/model/wall/walls.hpp"

namespace vcortex {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

}  // namespace

ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings) {
    const auto t0 = std::chrono::steady_clock::now();
    ModelResult res;
    ModelReport& rep = res.report;
    rep.points_in = snap.points.size();

    // cleaning
    std::vector<LidarPoint> pts = drop_outliers(snap.points, settings.outlier_mean_k, settings.outlier_stddev_mul);
    rep.after_outliers = pts.size();

    // floor; heights are taken above the plane, so the cloud is never levelled
    const Plane floor = find_floor(pts);
    rep.floor = floor;
    rep.tilt_deg = std::atan(std::hypot(floor.a, floor.b)) * kRadToDeg;
    const CandidateParams& cp = settings.candidates;
    // walls are read in a band above the floor and below the ceiling, where neither shows up
    const double band_lo = cp.band_lo;
    const double band_hi = settings.wall_height - cp.band_top_margin;

    // alignment
    const Grid raw = build_grid(pts, floor, band_lo, band_hi, cp.cell);
    const std::vector<Eigen::Vector2d> cells = band_cells(raw, cp.min_column);
    const double yaw = dominant_yaw(cells);
    rep.yaw_deg = yaw * kRadToDeg;
    // turns about z only, so heights need no fix
    const Eigen::Rotation2Dd to_aligned(-yaw);
    const Eigen::Rotation2Dd to_map(yaw);

    std::vector<LidarPoint> aligned = pts;
    for (LidarPoint& p : aligned) {
        p.position.head<2>() = (to_aligned * p.position.head<2>().cast<double>()).cast<float>();
    }
    Trajectory aligned_traj = traj;
    for (Pose& p : aligned_traj) {
        p.position.head<2>() = to_aligned * p.position.head<2>();
        p.yaw -= yaw;
    }
    // z = a x + b y + c holds in both frames once (a, b) turns with the points
    Plane aligned_floor = floor;
    const Eigen::Vector2d aligned_slope = to_aligned * Eigen::Vector2d(floor.a, floor.b);
    aligned_floor.a = aligned_slope.x();
    aligned_floor.b = aligned_slope.y();
    // rooms on the floor, then the outline of each: the wall faces met walking out of it
    const Grid grid = build_grid(aligned, aligned_floor, band_lo, band_hi, cp.cell);
    const FreeSpace space =
        find_floor_rooms(aligned, aligned_floor, grid, aligned_traj, settings.free_space);
    rep.free_space_area = space.area;
    rep.free_space_rooms = space.room_count;
    const std::vector<std::vector<OutlineSide>> room_outlines =
        find_room_outlines(space, grid, aligned, aligned_floor, band_hi, settings.outline);
    for (const std::vector<OutlineSide>& room_outline : room_outlines) {
        rep.outline_sides += room_outline.size();
    }

    // walls, then what is left in the band as objects, then the rooms the walls close
    WallsAndFurniture wo =
        find_walls(room_outlines, aligned, aligned_floor, band_lo, band_hi, settings.walls);
    std::vector<Object> objects =
        find_objects(wo.furniture_points, aligned_floor, settings.objects);
    std::vector<Room> rooms =
        find_rooms(wo.walls, grid, aligned_traj, settings.walls.door_max, settings.rooms);

    // room per object while both are still axis-aligned boxes
    for (Object& o : objects) {
        for (const Room& r : rooms) {
            if (r.closed && inside(o.center, r.outline)) {
                o.room_id = r.id;
                break;
            }
        }
    }

    // back to the map frame, so the building overlays the raw cloud as is
    for (Wall& w : wo.walls) {
        w.start = to_map * w.start;
        w.end = to_map * w.end;
    }
    for (Room& r : rooms) {
        for (Eigen::Vector2d& v : r.outline) v = to_map * v;
    }
    for (Object& o : objects) {
        o.center = to_map * o.center;
        o.yaw += yaw;
    }

    Level level;
    level.id = "level-1";
    // plane height under the middle of the walls, not c: c is the height at the map origin,
    // which may lie metres outside a tilted building
    Eigen::Vector2d mid = Eigen::Vector2d::Zero();
    for (const Eigen::Vector2d& v : cells) mid += v;
    if (!cells.empty()) mid /= static_cast<double>(cells.size());
    level.floor_z = floor.a * mid.x() + floor.b * mid.y() + floor.c;
    level.wall_height = settings.wall_height;
    level.wall_height_source = settings.wall_height_source;
    level.walls = std::move(wo.walls);
    level.rooms = std::move(rooms);
    level.objects = std::move(objects);

    rep.poses = traj.size();
    rep.walls = level.walls.size();
    rep.rooms = level.rooms.size();
    rep.objects = level.objects.size();

    res.building.run_id = snap.run_id;
    res.building.seq = snap.seq;
    res.building.frame_id = snap.frame_id;
    res.building.levels.push_back(std::move(level));
    res.cleaned = std::move(pts);
    rep.duration_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return res;
}

}  // namespace vcortex
