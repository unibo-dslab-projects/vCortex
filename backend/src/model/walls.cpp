#include "vcortex/model/walls.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <tuple>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl_bridge.hpp"

namespace vcortex {

namespace {

// one wall seen from two rooms gives two faces this far apart (5 cm cells, 5-35 cm walls)
constexpr double kPairMin = 0.05;
constexpr double kPairMax = 0.35;
constexpr double kEps = 1e-9;

double normal_of(Axis a, double x, double y) { return a == Axis::X ? x : y; }
double along_of(Axis a, double x, double y) { return a == Axis::X ? y : x; }

bool overlaps(double a0, double a1, double b0, double b1) { return a0 < b1 && b0 < a1; }

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
};

void count_poses(Face& f, const Trajectory& traj, const WallParams& p) {
    for (const Pose& pose : traj) {
        const double u = along_of(f.c.axis, pose.x, pose.y);
        const double d = normal_of(f.c.axis, pose.x, pose.y) - f.c.position;
        if (u < f.c.from - p.span_margin || u > f.c.to + p.span_margin ||
            std::abs(d) > p.both_sides_range || std::abs(d) < kEps) {
            continue;
        }
        ++(d < 0.0 ? f.lo : f.hi);
    }
    f.inside = f.lo > 0 && f.hi > 0;
    f.interior = f.hi > 0 ? 1 : f.lo > 0 ? -1 : 0;
}

Vec2 occupied_centre(const Grid& g) {
    double sx = 0.0, sy = 0.0, n = 0.0;
    for (int iy = 0; iy < g.height; ++iy) {
        for (int ix = 0; ix < g.width; ++ix) {
            if (g.column[static_cast<std::size_t>(iy) * static_cast<std::size_t>(g.width) +
                         static_cast<std::size_t>(ix)] > 0) {
                sx += g.origin.x + (ix + 0.5) * g.cell;
                sy += g.origin.y + (iy + 0.5) * g.cell;
                n += 1.0;
            }
        }
    }
    return n > 0.0 ? Vec2{sx / n, sy / n} : Vec2{};
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

// gap between a box side range [b0, b1] and the wall body around its centreline
double box_to_wall(double b0, double b1, const Piece& w) {
    const double w0 = w.centre - w.thickness / 2.0, w1 = w.centre + w.thickness / 2.0;
    return std::max({0.0, w0 - b1, b0 - w1});
}

double point_to_wall(double x, double y, const Piece& w) {
    const double dn =
        std::max(0.0, std::abs(normal_of(w.axis, x, y) - w.centre) - w.thickness / 2.0);
    const double u = along_of(w.axis, x, y);
    const double du = std::max({0.0, w.from - u, u - w.to});
    return std::hypot(dn, du);
}

}  // namespace

WallsAndObjects split_walls_objects(const std::vector<Candidate>& cands, const Grid& grid,
                                    const std::vector<Point>& aligned, const Plane& floor,
                                    double band_lo, double band_hi,
                                    const Trajectory& aligned_traj, const WallParams& p) {
    std::vector<Face> faces;
    faces.reserve(cands.size());
    for (const Candidate& c : cands) {
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
    const Vec2 middle = occupied_centre(grid);
    for (Face& f : faces) {
        if (!f.paired && !f.inside && f.interior == 0) {
            f.interior = normal_of(f.c.axis, middle.x, middle.y) >= f.c.position ? 1 : -1;
        }
    }
    for (Face& f : faces) {
        if (!f.paired && !f.inside) f.object_front = has_wall_behind(f, faces, p);
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

    std::vector<Piece> walls = join_collinear(std::move(pieces), faces, p);
    close_corners(walls, p);
    std::sort(walls.begin(), walls.end(), [](const Piece& a, const Piece& b) {
        return std::tie(a.axis, a.centre, a.from) < std::tie(b.axis, b.centre, b.from);
    });

    WallsAndObjects out;
    for (std::size_t i = 0; i < walls.size(); ++i) {
        const Piece& w = walls[i];
        Wall wall;
        wall.id = "wall-" + std::to_string(i + 1);
        wall.start = w.axis == Axis::X ? Vec2{w.centre, w.from} : Vec2{w.from, w.centre};
        wall.end = w.axis == Axis::X ? Vec2{w.centre, w.to} : Vec2{w.to, w.centre};
        wall.thickness = w.thickness;
        const double length = w.to - w.from;
        wall.coverage = length > 0.0
                            ? std::min(1.0, static_cast<double>(w.cells) * grid.cell / length)
                            : 0.0;
        out.walls.push_back(wall);
    }

    // band points near a wall body count for that wall; the rest are furniture
    std::vector<Point> rest;
    for (const Point& pt : aligned) {
        const double h = height_above(floor, pt);
        if (h < band_lo || h >= band_hi) continue;
        std::size_t nearest = walls.size();
        double nearest_d = p.object_clearance + kEps;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            const double d = point_to_wall(pt.x, pt.y, walls[i]);
            if (d <= nearest_d) {
                nearest = i;
                nearest_d = d;
            }
        }
        if (nearest < walls.size()) {
            ++out.walls[nearest].points;
        } else {
            rest.push_back(pt);
        }
    }
    if (rest.empty()) return out;

    const auto cloud = to_pcl(rest);
    const auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(p.cluster_tolerance);
    ec.setMinClusterSize(p.cluster_min_points);
    ec.setMaxClusterSize(std::numeric_limits<int>::max());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    ec.extract(clusters);

    for (const pcl::PointIndices& cl : clusters) {
        const std::vector<Point> pts = pick(rest, cl.indices);
        float x0 = pts[0].x, x1 = x0, y0 = pts[0].y, y1 = y0, z0 = pts[0].z, z1 = z0;
        for (const Point& q : pts) {
            x0 = std::min(x0, q.x);
            x1 = std::max(x1, q.x);
            y0 = std::min(y0, q.y);
            y1 = std::max(y1, q.y);
            z0 = std::min(z0, q.z);
            z1 = std::max(z1, q.z);
        }
        Object obj;
        obj.center = {(x0 + x1) / 2.0, (y0 + y1) / 2.0};
        obj.size = {static_cast<double>(x1 - x0), static_cast<double>(y1 - y0)};
        obj.bottom_z = z0;
        obj.top_z_seen = z1;
        obj.points = pts.size();
        double best = p.against_wall_distance + kEps;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            const Piece& w = walls[i];
            const bool along = w.axis == Axis::X ? overlaps(y0, y1, w.from, w.to)
                                                 : overlaps(x0, x1, w.from, w.to);
            const double d = w.axis == Axis::X ? box_to_wall(x0, x1, w) : box_to_wall(y0, y1, w);
            if (along && d <= best) {
                best = d;
                obj.against_wall_id = out.walls[i].id;
            }
        }
        out.objects.push_back(obj);
    }
    std::sort(out.objects.begin(), out.objects.end(), [](const Object& a, const Object& b) {
        return std::tie(a.center.x, a.center.y) < std::tie(b.center.x, b.center.y);
    });
    for (std::size_t i = 0; i < out.objects.size(); ++i) {
        out.objects[i].id = "object-" + std::to_string(i + 1);
    }
    return out;
}

}  // namespace vcortex
