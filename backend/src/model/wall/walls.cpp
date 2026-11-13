#include "vcortex/model/wall/walls.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <tuple>

namespace vcortex {

namespace {

// one wall seen from two rooms gives two faces this far apart (5 cm cells, 5-35 cm walls)
constexpr double kPairMin = 0.05;
constexpr double kPairMax = 0.35;
constexpr double kEps = 1e-9;

// interior: +1 room at larger normal coordinate, -1 at smaller, 0 both (wall seen from two rooms)
struct Face {
    Candidate c;
    std::size_t lo = 0, hi = 0;  // poses on each side
    int interior = 0;
    bool inside = false;         // poses on both sides: stands in a room
    bool paired = false;
    bool object_front = false;
};

struct Piece {
    Axis axis = Axis::X;
    double centre = 0.0, thickness = 0.0;
    double from = 0.0, to = 0.0;
    int interior = 0;
    std::size_t cells = 0;
    bool is_shell = false;  // came from an outline side
};

void count_poses(Face& f, const Trajectory& traj, const WallParams& p) {
    for (const Pose& pose : traj) {
        const double u = along_of(f.c.axis, pose.position.x(), pose.position.y());
        const double d = normal_of(f.c.axis, pose.position.x(), pose.position.y()) - f.c.position;
        if (u < f.c.from - p.span_margin || u > f.c.to + p.span_margin ||
            std::abs(d) > p.both_sides_range || std::abs(d) < kEps) {
            continue;
        }
        ++(d < 0.0 ? f.lo : f.hi);
    }
    f.inside = f.lo > 0 && f.hi > 0;
    f.interior = f.hi > 0 ? 1 : f.lo > 0 ? -1 : 0;
}

Eigen::Vector2d occupied_centre(const Grid& g) {
    double sx = 0.0, sy = 0.0, n = 0.0;
    for (int iy = 0; iy < g.height; ++iy) {
        for (int ix = 0; ix < g.width; ++ix) {
            if (g.column[static_cast<std::size_t>(iy) * static_cast<std::size_t>(g.width) +
                         static_cast<std::size_t>(ix)] > 0) {
                sx += g.origin.x() + (ix + 0.5) * g.cell;
                sy += g.origin.y() + (iy + 0.5) * g.cell;
                n += 1.0;
            }
        }
    }
    return n > 0.0 ? Eigen::Vector2d{sx / n, sy / n} : Eigen::Vector2d::Zero();
}

// pairs first: each face of a wall between two walked rooms has poses on both sides, which
// would otherwise read as furniture standing in a room
void pair_faces(std::vector<Face>& faces, std::vector<Piece>& pieces) {
    for (std::size_t i = 0; i < faces.size(); ++i) {
        Face& a = faces[i];
        if (a.paired) continue;
        std::size_t best = faces.size();
        double best_d = std::numeric_limits<double>::max();
        for (std::size_t j = 0; j < faces.size(); ++j) {
            const Face& b = faces[j];
            const double d = b.c.position - a.c.position;
            if (j == i || b.paired || b.c.axis != a.c.axis || d < kPairMin - kEps ||
                d > kPairMax + kEps || !overlaps(a.c.from, a.c.to, b.c.from, b.c.to)) {
                continue;
            }
            // a has its room below, b above: poses beyond both outer sides
            if (a.lo > 0 && b.hi > 0 && d < best_d) {
                best = j;
                best_d = d;
            }
        }
        if (best == faces.size()) continue;
        Face& b = faces[best];
        a.paired = b.paired = true;
        Piece w;
        w.axis = a.c.axis;
        w.centre = (a.c.position + b.c.position) / 2.0;
        w.thickness = best_d;
        w.from = std::min(a.c.from, b.c.from);
        w.to = std::max(a.c.to, b.c.to);
        w.interior = 0;
        w.cells = std::max(a.c.cells, b.c.cells);
        pieces.push_back(w);
    }
}

// a thin wall between two walked rooms shows both faces in neighbouring cells, which merge
// into one face with poses on both sides; furniture walked around shows a second face deeper
bool has_far_side(const Face& f, const std::vector<Face>& faces, const WallParams& p) {
    for (const Face& b : faces) {
        const double d = std::abs(b.c.position - f.c.position);
        if (&b != &f && b.c.axis == f.c.axis && d > kPairMax && d <= p.max_depth + kEps &&
            overlaps(f.c.from, f.c.to, b.c.from, b.c.to)) {
            return true;
        }
    }
    return false;
}

// wardrobe rule: a parallel face behind f (away from its room), close and overlapping, is
// the wall, so f is the front of something standing against it
bool has_wall_behind(const Face& f, const std::vector<Face>& faces, const WallParams& p) {
    for (const Face& b : faces) {
        if (&b == &f || b.c.axis != f.c.axis || b.inside) continue;
        const double depth = (b.c.position - f.c.position) * -f.interior;
        const bool same_room = b.paired || b.interior == f.interior;
        if (depth > kEps && depth <= p.max_depth + kEps && same_room &&
            overlaps(f.c.from - p.overlap_widen, f.c.to + p.overlap_widen, b.c.from, b.c.to)) {
            return true;
        }
    }
    return false;
}

// an object front standing in front of the wall across [g0, g1] hides that stretch
bool gap_hidden(const Piece& w, double g0, double g1, const std::vector<Face>& faces,
                const WallParams& p) {
    for (const Face& f : faces) {
        if (!f.object_front || f.c.axis != w.axis) continue;
        const double d = f.c.position - w.centre;
        const double ahead = w.interior == 0 ? std::abs(d) : d * w.interior;
        if (ahead > 0.0 && ahead <= p.max_depth && f.c.from <= g0 + p.min_gap &&
            f.c.to >= g1 - p.min_gap) {
            return true;
        }
    }
    return false;
}

std::vector<Piece> join_collinear(std::vector<Piece> pieces, const std::vector<Face>& faces,
                                  const WallParams& p) {
    std::sort(pieces.begin(), pieces.end(), [](const Piece& a, const Piece& b) {
        return std::tie(a.axis, a.interior, a.centre) < std::tie(b.axis, b.interior, b.centre);
    });
    std::vector<Piece> out;
    std::size_t i = 0;
    while (i < pieces.size()) {
        // one line: same axis and room side, centres chained within collinear_tolerance
        std::size_t j = i + 1;
        while (j < pieces.size() && pieces[j].axis == pieces[i].axis &&
               pieces[j].interior == pieces[i].interior &&
               pieces[j].centre - pieces[j - 1].centre <= p.collinear_tolerance + kEps) {
            ++j;
        }
        std::vector<Piece> line(pieces.begin() + static_cast<std::ptrdiff_t>(i),
                                pieces.begin() + static_cast<std::ptrdiff_t>(j));
        std::sort(line.begin(), line.end(),
                  [](const Piece& a, const Piece& b) { return a.from < b.from; });
        Piece cur = line.front();
        double weight = static_cast<double>(cur.cells);
        double centre_sum = cur.centre * weight, thick_sum = cur.thickness * weight;
        auto flush = [&] {
            if (weight > 0.0) {
                cur.centre = centre_sum / weight;
                cur.thickness = thick_sum / weight;
            }
            out.push_back(cur);
        };
        for (std::size_t k = 1; k < line.size(); ++k) {
            const Piece& n = line[k];
            const double gap = n.from - cur.to;
            // longer gaps not hidden by furniture stay open: doors, or wall never seen
            if (gap <= p.min_gap + kEps || gap_hidden(cur, cur.to, n.from, faces, p)) {
                cur.to = std::max(cur.to, n.to);
                cur.cells += n.cells;
                cur.is_shell = cur.is_shell || n.is_shell;
                const double nw = static_cast<double>(n.cells);
                centre_sum += n.centre * nw;
                thick_sum += n.thickness * nw;
                weight += nw;
            } else {
                flush();
                cur = n;
                weight = static_cast<double>(n.cells);
                centre_sum = n.centre * weight;
                thick_sum = n.thickness * weight;
            }
        }
        flush();
        i = j;
    }
    return out;
}

// ends snap to the centreline of a perpendicular wall within corner_gap, extending or trimming
void close_corners(std::vector<Piece>& walls, const WallParams& p) {
    const std::vector<Piece> before = walls;
    for (Piece& w : walls) {
        for (double* end : {&w.from, &w.to}) {
            double best = std::numeric_limits<double>::max();
            for (const Piece& q : before) {
                if (q.axis == w.axis || w.centre < q.from - p.corner_gap ||
                    w.centre > q.to + p.corner_gap) {
                    continue;
                }
                const double d = std::abs(q.centre - *end);
                if (d <= p.corner_gap + kEps && d < std::abs(best - *end)) best = q.centre;
            }
            if (best != std::numeric_limits<double>::max()) *end = best;
        }
        if (w.from > w.to) std::swap(w.from, w.to);
    }
}

// true when a point lies beyond a side of this axis whose span holds its position along the wall,
// past the face by more than margin; a negative margin also counts points in front of the face
bool is_beyond_outline_side(Axis axis, double normal_coordinate, double along_coordinate,
                            const std::vector<OutlineSide>& outline, double margin) {
    for (const OutlineSide& side : outline) {
        if (side.direction.axis != axis || along_coordinate < side.from ||
            along_coordinate > side.to) {
            continue;
        }
        const double depth_beyond_face = (normal_coordinate - side.face) * side.direction.outward;
        if (depth_beyond_face > -margin) return true;
    }
    return false;
}

// a candidate facing the room, within max_depth in front of a side: that side is the wall behind
bool is_in_front_of_outline(const Face& candidate_face, const std::vector<OutlineSide>& outline,
                            const WallParams& p) {
    for (const OutlineSide& side : outline) {
        if (side.direction.axis != candidate_face.c.axis) continue;
        if (!overlaps(candidate_face.c.from - p.overlap_widen, candidate_face.c.to + p.overlap_widen,
                      side.from, side.to)) {
            continue;
        }
        if (candidate_face.interior != -side.direction.outward) continue;
        const double depth_in_front =
            (side.face - candidate_face.c.position) * side.direction.outward;
        if (depth_in_front > kEps && depth_in_front <= p.max_depth + kEps) return true;
    }
    return false;
}

double point_to_wall(double x, double y, const Piece& w) {
    const double dn =
        std::max(0.0, std::abs(normal_of(w.axis, x, y) - w.centre) - w.thickness / 2.0);
    const double u = along_of(w.axis, x, y);
    const double du = std::max({0.0, w.from - u, u - w.to});
    return std::hypot(dn, du);
}

// a door may leave door_max open where a wall meets the shell or goes on in line; against
// another inside wall the end must touch (corner_gap), or rows of desks chain to each other
bool ends_meet(const Piece& wall, const Piece& other, const WallParams& p) {
    const bool is_in_line = other.axis == wall.axis &&
                            std::abs(other.centre - wall.centre) <= p.collinear_tolerance + kEps;
    const double allowed_gap = is_in_line || other.is_shell ? p.door_max : p.corner_gap;
    for (const double end : {wall.from, wall.to}) {
        const double x = wall.axis == Axis::X ? wall.centre : end;
        const double y = wall.axis == Axis::X ? end : wall.centre;
        if (point_to_wall(x, y, other) <= allowed_gap + kEps) return true;
    }
    return false;
}

// with a room shell, a wall inside it hangs from the shell, directly or through other walls;
// desks walked around show faces on both sides but float free, so their points go to objects
void keep_walls_hanging_from_shell(std::vector<Piece>& walls, const WallParams& p) {
    std::vector<bool> is_hanging(walls.size());
    for (std::size_t i = 0; i < walls.size(); ++i) is_hanging[i] = walls[i].is_shell;
    bool found_one = true;
    while (found_one) {
        found_one = false;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            if (is_hanging[i]) continue;
            for (std::size_t j = 0; j < walls.size() && !is_hanging[i]; ++j) {
                if (is_hanging[j] &&
                    (ends_meet(walls[i], walls[j], p) || ends_meet(walls[j], walls[i], p))) {
                    is_hanging[i] = true;
                    found_one = true;
                }
            }
        }
    }
    std::vector<Piece> kept;
    for (std::size_t i = 0; i < walls.size(); ++i) {
        if (is_hanging[i]) kept.push_back(walls[i]);
    }
    walls = std::move(kept);
}

// 95th percentile of band heights near the wall body, so one stray high point does not set it
double seen_top(const Piece& wall, const std::vector<LidarPoint>& aligned, const Plane& floor,
                double band_lo, double band_hi, const WallParams& p) {
    std::vector<double> heights;
    for (const LidarPoint& point : aligned) {
        const double height = height_above(floor, point);
        if (height < band_lo || height >= band_hi) continue;
        if (point_to_wall(point.position.x(), point.position.y(), wall) <=
            p.object_clearance + kEps) {
            heights.push_back(height);
        }
    }
    if (heights.empty()) return 0.0;
    const auto top_index =
        static_cast<std::ptrdiff_t>(0.95 * static_cast<double>(heights.size() - 1));
    std::nth_element(heights.begin(), heights.begin() + top_index, heights.end());
    return heights[static_cast<std::size_t>(top_index)];
}

// a desk walked around passes every rule by shape; only its height gives it away. the lidar
// never sees the true top, so inside walls are compared with what it saw of the shell
double drop_low_walls(std::vector<Piece>& walls, const std::vector<LidarPoint>& aligned,
                      const Plane& floor, double band_lo, double band_hi, const WallParams& p) {
    std::vector<double> tops(walls.size());
    double shell_top_sum = 0.0;
    int shell_walls_seen = 0;
    for (std::size_t i = 0; i < walls.size(); ++i) {
        tops[i] = seen_top(walls[i], aligned, floor, band_lo, band_hi, p);
        if (walls[i].is_shell && tops[i] > 0.0) {
            shell_top_sum += tops[i];
            ++shell_walls_seen;
        }
    }
    if (shell_walls_seen == 0) return 0.0;
    const double shell_top_seen = shell_top_sum / shell_walls_seen;
    std::vector<Piece> kept;
    for (std::size_t i = 0; i < walls.size(); ++i) {
        if (walls[i].is_shell || tops[i] >= p.min_top_ratio * shell_top_seen) {
            kept.push_back(walls[i]);
        }
    }
    walls = std::move(kept);
    return shell_top_seen;
}

}  // namespace

WallsAndFurniture find_walls(const std::vector<Candidate>& cands,
                                    const std::vector<OutlineSide>& outline, const Grid& grid,
                                    const std::vector<LidarPoint>& aligned, const Plane& floor,
                                    double band_lo, double band_hi,
                                    const Trajectory& aligned_traj, const WallParams& p) {
    std::vector<Face> faces;
    faces.reserve(cands.size());
    for (const Candidate& c : cands) {
        // on the shell or past it: the outline already is that wall
        const double middle_along = (c.from + c.to) / 2.0;
        if (is_beyond_outline_side(c.axis, c.position, middle_along, outline, p.object_clearance)) {
            continue;
        }
        Face f;
        f.c = c;
        count_poses(f, aligned_traj, p);
        faces.push_back(f);
    }
    std::sort(faces.begin(), faces.end(), [](const Face& a, const Face& b) {
        return std::tie(a.c.axis, a.c.position, a.c.from) <
               std::tie(b.c.axis, b.c.position, b.c.from);
    });

    std::vector<Piece> pieces;
    pair_faces(faces, pieces);
    for (Face& f : faces) {
        if (f.paired || !f.inside || has_far_side(f, faces, p)) continue;
        f.paired = true;
        Piece w;
        w.axis = f.c.axis;
        w.centre = f.c.position;
        w.thickness = p.thickness;
        w.from = f.c.from;
        w.to = f.c.to;
        w.cells = f.c.cells;
        pieces.push_back(w);
    }

    // no poses near a face (or no trajectory at all): guess its room is towards the middle of
    // the map, so the outermost of overlapping parallel faces becomes the wall
    const Eigen::Vector2d middle = occupied_centre(grid);
    for (Face& f : faces) {
        if (!f.paired && !f.inside && f.interior == 0) {
            f.interior = normal_of(f.c.axis, middle.x(), middle.y()) >= f.c.position ? 1 : -1;
        }
    }
    for (Face& f : faces) {
        if (f.paired || f.inside) continue;
        f.object_front =
            is_in_front_of_outline(f, outline, p) || has_wall_behind(f, faces, p);
    }
    for (const Face& f : faces) {
        if (f.paired || f.inside || f.object_front) continue;
        Piece w;
        w.axis = f.c.axis;
        w.thickness = p.thickness;
        // the face seen is the room side; the centreline lies half a thickness behind it
        w.centre = f.c.position - f.interior * p.thickness / 2.0;
        w.from = f.c.from;
        w.to = f.c.to;
        w.interior = f.interior;
        w.cells = f.c.cells;
        pieces.push_back(w);
    }

    // each outline side is a wall; its seen cells say how much of it the robot saw
    for (const OutlineSide& side : outline) {
        Piece shell_wall;
        shell_wall.axis = side.direction.axis;
        shell_wall.thickness = p.thickness;
        // the face seen is the room side; the centreline lies half a thickness outward
        shell_wall.centre = side.face + side.direction.outward * p.thickness / 2.0;
        shell_wall.from = side.from;
        shell_wall.to = side.to;
        shell_wall.interior = -side.direction.outward;
        shell_wall.cells = side.seen_cell_count;
        shell_wall.is_shell = true;
        pieces.push_back(shell_wall);
    }

    std::vector<Piece> walls = join_collinear(std::move(pieces), faces, p);
    close_corners(walls, p);
    WallsAndFurniture out;
    if (!outline.empty()) {
        // height first: a wall that hung only from a dropped desk then floats too
        out.shell_top_seen = drop_low_walls(walls, aligned, floor, band_lo, band_hi, p);
        keep_walls_hanging_from_shell(walls, p);
    }
    std::sort(walls.begin(), walls.end(), [](const Piece& a, const Piece& b) {
        return std::tie(a.axis, a.centre, a.from) < std::tie(b.axis, b.centre, b.from);
    });

    for (std::size_t i = 0; i < walls.size(); ++i) {
        const Piece& w = walls[i];
        Wall wall;
        wall.id = "wall-" + std::to_string(i + 1);
        wall.start = w.axis == Axis::X ? Eigen::Vector2d{w.centre, w.from}
                                        : Eigen::Vector2d{w.from, w.centre};
        wall.end = w.axis == Axis::X ? Eigen::Vector2d{w.centre, w.to}
                                      : Eigen::Vector2d{w.to, w.centre};
        wall.thickness = w.thickness;
        const double length = w.to - w.from;
        wall.coverage = length > 0.0
                            ? std::min(1.0, static_cast<double>(w.cells) * grid.cell / length)
                            : 0.0;
        out.walls.push_back(wall);
    }

    // band points near a wall body count for that wall; the rest are furniture
    std::vector<LidarPoint> rest;
    for (const LidarPoint& pt : aligned) {
        const double h = height_above(floor, pt);
        if (h < band_lo || h >= band_hi) continue;
        std::size_t nearest = walls.size();
        double nearest_d = p.object_clearance + kEps;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            const double d = point_to_wall(pt.position.x(), pt.position.y(), walls[i]);
            if (d <= nearest_d) {
                nearest = i;
                nearest_d = d;
            }
        }
        if (nearest < walls.size()) {
            ++out.walls[nearest].points;
        } else if (!is_beyond_outline_side(Axis::X, pt.position.x(), pt.position.y(), outline,
                                           -p.object_clearance) &&
                   !is_beyond_outline_side(Axis::Y, pt.position.y(), pt.position.x(), outline,
                                           -p.object_clearance)) {
            // points past the room shell lie outside the room, so they are not furniture
            rest.push_back(pt);
        }
    }
    out.furniture_points = std::move(rest);
    return out;
}

}  // namespace vcortex
