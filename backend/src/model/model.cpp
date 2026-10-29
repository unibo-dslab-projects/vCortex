#include "vcortex/model/model.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "vcortex/model/align.hpp"
#include "vcortex/model/candidates.hpp"
#include "vcortex/model/clean.hpp"
#include "vcortex/model/grid.hpp"
#include "vcortex/model/rooms.hpp"
#include "vcortex/model/walls.hpp"

namespace vcortex {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

// turn about z by the angle whose cos and sin are given; z stays, so heights need no fix
struct Turn {
    double c = 1.0, s = 0.0;
    Vec2 operator()(double x, double y) const { return {c * x - s * y, s * x + c * y}; }
    Vec2 operator()(const Vec2& v) const { return (*this)(v.x, v.y); }
};

bool inside(const Vec2& p, const std::vector<Vec2>& box) {
    return p.x >= box[0].x && p.x <= box[2].x && p.y >= box[0].y && p.y <= box[2].y;
}

}  // namespace

ModelResult run_model(const Snapshot& snap, const Trajectory& traj, const ModelSettings& settings) {
    const auto t0 = std::chrono::steady_clock::now();
    ModelResult res;
    ModelReport& rep = res.report;
    res.has_hits = snap.has_hits;
    res.has_intensity = snap.has_intensity;
    rep.points_in = snap.points.size();

    // A: cleaning; old map.pcd files carry no hits, and 0 would drop every point
    std::vector<Point> pts;
    if (snap.has_hits) {
        pts = drop_transient(snap.points, settings.min_scans);
    } else {
        pts = snap.points;
        rep.transient_skipped = true;
    }
    rep.after_transient = pts.size();
    pts = drop_outliers(pts, settings.outlier_mean_k, settings.outlier_stddev_mul);
    rep.after_outliers = pts.size();

    // B: floor; heights are taken above the plane, so the cloud is never levelled
    const Plane floor = find_floor(pts);
    rep.floor = floor;
    rep.tilt_deg = std::atan(std::hypot(floor.a, floor.b)) * kRadToDeg;
    const CandidateParams& cp = settings.candidates;
    const double band_lo = cp.band_lo;
    const double band_hi = settings.wall_height - cp.band_top_margin;

    // C: alignment
    const Grid raw = build_grid(pts, floor, band_lo, band_hi, cp.cell);
    const std::vector<Vec2> cells = band_cells(raw, cp.min_column);
    const double yaw = dominant_yaw(cells);
    rep.yaw_deg = yaw * kRadToDeg;
    const Turn to_aligned{std::cos(yaw), -std::sin(yaw)};
    const Turn to_map{std::cos(yaw), std::sin(yaw)};

    std::vector<Point> aligned = pts;
    for (Point& p : aligned) {
        const Vec2 v = to_aligned(p.x, p.y);
        p.x = static_cast<float>(v.x);
        p.y = static_cast<float>(v.y);
    }
    Trajectory aligned_traj = traj;
    for (Pose& p : aligned_traj) {
        const Vec2 v = to_aligned(p.x, p.y);
        p.x = v.x;
        p.y = v.y;
        p.yaw -= yaw;
    }
    // z = a x + b y + c holds in both frames once (a, b) turns with the points
    Plane aligned_floor = floor;
    aligned_floor.a = to_aligned.c * floor.a - to_aligned.s * floor.b;
    aligned_floor.b = to_aligned.s * floor.a + to_aligned.c * floor.b;
    const Grid grid = build_grid(aligned, aligned_floor, band_lo, band_hi, cp.cell);

    // D, E, F
    const std::vector<Candidate> cands = find_candidates(grid, cp);
    WallsAndObjects wo = split_walls_objects(cands, grid, aligned, aligned_floor, band_lo, band_hi,
                                             aligned_traj, settings.walls);
    std::vector<Room> rooms =
        find_rooms(wo.walls, grid, aligned_traj, settings.walls.door_max, settings.rooms);

    // room per object while both are still axis-aligned boxes
    for (Object& o : wo.objects) {
        for (const Room& r : rooms) {
            if (r.closed && inside(o.center, r.outline)) {
                o.room_id = r.id;
                break;
            }
        }
    }

    // G: back to the map frame, so the building overlays the raw cloud as is
    for (Wall& w : wo.walls) {
        w.start = to_map(w.start);
        w.end = to_map(w.end);
    }
    for (Room& r : rooms) {
        for (Vec2& v : r.outline) v = to_map(v);
    }
    for (Object& o : wo.objects) {
        o.center = to_map(o.center);
        o.yaw += yaw;
    }

    Level level;
    level.id = "level-1";
    // plane height under the middle of the walls, not c: c is the height at the map origin,
    // which may lie metres outside a tilted building
    Vec2 mid;
    for (const Vec2& v : cells) {
        mid.x += v.x;
        mid.y += v.y;
    }
    if (!cells.empty()) {
        mid.x /= static_cast<double>(cells.size());
        mid.y /= static_cast<double>(cells.size());
    }
    level.floor_z = floor.a * mid.x + floor.b * mid.y + floor.c;
    level.wall_height = settings.wall_height;
    level.wall_height_source = settings.wall_height_source;
    level.walls = std::move(wo.walls);
    level.rooms = std::move(rooms);
    level.objects = std::move(wo.objects);

    rep.poses = traj.size();
    rep.candidates = cands.size();
    rep.walls = level.walls.size();
    rep.rooms = level.rooms.size();
    rep.objects = level.objects.size();

    res.building.run_id = snap.run_id;
    res.building.seq = snap.seq;
    res.building.levels.push_back(std::move(level));
    res.cleaned = std::move(pts);
    rep.duration_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return res;
}

}  // namespace vcortex
