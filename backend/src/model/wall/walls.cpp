#include "vcortex/model/wall/walls.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <tuple>
#include <utility>

#include "vcortex/geometry/vectors.hpp"

namespace vcortex {

namespace {

// one wall seen from two rooms gives two faces this far apart (5 cm cells, 5-35 cm walls)
constexpr double kPairMin = 0.05;
constexpr double kPairMax = 0.35;
constexpr double kEps = 1e-9;

/// A stretch of wall before joining: centerline, thickness and extent along it, metres.
struct Piece {
    Axis axis = Axis::X;
    double center = 0.0, thickness = 0.0;
    double from = 0.0, to = 0.0;
    // +1 room at larger normal coordinate, -1 at smaller, 0 rooms on both sides
    int interior = 0;
};

double piece_length(const Piece& piece) { return piece.to - piece.from; }

/// Index of the side in sides that is the other face of the wall behind side, or sides.size():
/// a side of the same axis facing back, kPairMin to kPairMax behind side's face, overlapping it.
/// The nearest wins.
std::size_t find_other_face(const OutlineSide& side, const std::vector<OutlineSide>& sides,
                            const std::vector<bool>& is_paired) {
    std::size_t other_face = sides.size();
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t j = 0; j < sides.size(); ++j) {
        const OutlineSide& other = sides[j];
        if (is_paired[j] || other.direction.axis != side.direction.axis ||
            other.direction.outward != -side.direction.outward ||
            !overlaps(side.from, side.to, other.from, other.to)) {
            continue;
        }
        const double distance = (other.face - side.face) * side.direction.outward;
        if (distance < kPairMin - kEps || distance > kPairMax + kEps) continue;
        if (distance < nearest_distance) {
            nearest_distance = distance;
            other_face = j;
        }
    }
    return other_face;
}

/// One piece per wall: two faces of one wall between two rooms make one piece as thick as their
/// distance; a side alone makes a piece wall_thickness thick, half of it behind the face.
std::vector<Piece> make_pieces(const std::vector<OutlineSide>& sides, double wall_thickness) {
    std::vector<Piece> pieces;
    std::vector<bool> is_paired(sides.size(), false);
    for (std::size_t i = 0; i < sides.size(); ++i) {
        if (is_paired[i]) continue;
        is_paired[i] = true;
        const OutlineSide& side = sides[i];
        Piece piece;
        piece.axis = side.direction.axis;
        const std::size_t other_face = find_other_face(side, sides, is_paired);
        if (other_face < sides.size()) {
            is_paired[other_face] = true;
            const OutlineSide& other = sides[other_face];
            piece.center = (side.face + other.face) / 2.0;
            piece.thickness = std::abs(other.face - side.face);
            piece.from = std::min(side.from, other.from);
            piece.to = std::max(side.to, other.to);
        } else {
            piece.center = side.face + side.direction.outward * wall_thickness / 2.0;
            piece.thickness = wall_thickness;
            piece.from = side.from;
            piece.to = side.to;
            piece.interior = -side.direction.outward;
        }
        pieces.push_back(piece);
    }
    return pieces;
}

/// The pieces split into lines: the longest piece left opens a line, and every piece left with
/// its axis and side, and its center within collinear_tolerance of the longest's, joins it.
/// The longest leads because it was seen over the most metres: a short stray piece never
/// decides where a line is, and a chain of pieces each near the next never grows past it.
std::vector<std::vector<Piece>> group_into_lines(std::vector<Piece> pieces,
                                                 double collinear_tolerance) {
    std::stable_sort(pieces.begin(), pieces.end(), [](const Piece& a, const Piece& b) {
        return piece_length(a) > piece_length(b);
    });
    std::vector<std::vector<Piece>> lines;
    std::vector<bool> is_grouped(pieces.size(), false);
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        if (is_grouped[i]) continue;
        const Piece& longest = pieces[i];
        std::vector<Piece> line;
        for (std::size_t j = i; j < pieces.size(); ++j) {
            const Piece& piece = pieces[j];
            if (is_grouped[j] || piece.axis != longest.axis || piece.interior != longest.interior ||
                std::abs(piece.center - longest.center) > collinear_tolerance + kEps) {
                continue;
            }
            is_grouped[j] = true;
            line.push_back(piece);
        }
        lines.push_back(std::move(line));
    }
    return lines;
}

/// Joins the pieces of each line across gaps up to min_gap into one wall; center and thickness
/// become the means weighted by length.
std::vector<Piece> join_collinear(std::vector<Piece> pieces, const WallParams& p) {
    std::vector<Piece> out;
    for (std::vector<Piece>& line : group_into_lines(std::move(pieces), p.collinear_tolerance)) {
        std::sort(line.begin(), line.end(),
                  [](const Piece& a, const Piece& b) { return a.from < b.from; });
        Piece cur = line.front();
        double weight = piece_length(cur);
        double center_sum = cur.center * weight, thick_sum = cur.thickness * weight;
        auto flush = [&] {
            if (weight > 0.0) {
                cur.center = center_sum / weight;
                cur.thickness = thick_sum / weight;
            }
            out.push_back(cur);
        };
        for (std::size_t k = 1; k < line.size(); ++k) {
            const Piece& n = line[k];
            // merging the close pieces
            if (n.from - cur.to <= p.min_gap + kEps) {
                cur.to = std::max(cur.to, n.to);
                const double nw = piece_length(n);
                center_sum += n.center * nw;
                thick_sum += n.thickness * nw;
                weight += nw;
            } else {
                // longer gaps stay open: doors, or wall never seen
                flush();
                cur = n;
                weight = piece_length(n);
                center_sum = n.center * weight;
                thick_sum = n.thickness * weight;
            }
        }
        flush();
    }
    return out;
}

/// Closes corners of wall pieces by snapping their ends to the centerline of a perpendicular wall
/// within corner_gap, extending or trimming them as needed.
void close_corners(std::vector<Piece>& walls, const WallParams& p) {
    const std::vector<Piece> before = walls;
    for (Piece& w : walls) {
        for (double* end : {&w.from, &w.to}) {
            double best = std::numeric_limits<double>::max();
            for (const Piece& q : before) {
                if (q.axis == w.axis || w.center < q.from - p.corner_gap ||
                    w.center > q.to + p.corner_gap) {
                    continue;
                }
                const double d = std::abs(q.center - *end);
                if (d <= p.corner_gap + kEps && d < std::abs(best - *end)) best = q.center;
            }
            if (best != std::numeric_limits<double>::max()) *end = best;
        }
        if (w.from > w.to) std::swap(w.from, w.to);
    }
}

// true when a point lies beyond a side of this axis whose span holds its position along the wall;
// a negative margin only counts points more than -margin past the face
bool is_out_outline_side(Axis axis, double normal_coordinate, double along_coordinate,
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

// inside when past no side of the outline by more than clearance, on either axis
bool is_inside_any_outline(double x, double y,
                           const std::vector<std::vector<OutlineSide>>& room_outlines,
                           double clearance) {
    for (const std::vector<OutlineSide>& outline : room_outlines) {
        if (!is_out_outline_side(Axis::X, x, y, outline, -clearance) &&
            !is_out_outline_side(Axis::Y, y, x, outline, -clearance)) {
            return true;
        }
    }
    return false;
}

/// Distance from a point to the wall body, a box thickness wide; 0 inside it.
double distance_from_point_to_wall(double x, double y, const Piece& w) {
    const double dn =
        std::max(0.0, std::abs(normal_of(w.axis, x, y) - w.center) - w.thickness / 2.0);
    const double u = along_of(w.axis, x, y);
    const double du = std::max({0.0, w.from - u, u - w.to});
    return std::hypot(dn, du);
}

}  // namespace

WallsAndFurniture find_walls(const std::vector<std::vector<OutlineSide>>& room_outlines,
                             const std::vector<LidarPoint>& aligned, const Plane& floor,
                             double band_lo, double band_hi, const WallParams& p) {
    std::vector<OutlineSide> sides;
    for (const std::vector<OutlineSide>& outline : room_outlines) {
        sides.insert(sides.end(), outline.begin(), outline.end());
    }
    std::vector<Piece> walls = join_collinear(make_pieces(sides, p.thickness), p);
    close_corners(walls, p);
    std::sort(walls.begin(), walls.end(), [](const Piece& a, const Piece& b) {
        return std::tie(a.axis, a.center, a.from) < std::tie(b.axis, b.center, b.from);
    });

    WallsAndFurniture out;
    for (std::size_t i = 0; i < walls.size(); ++i) {
        const Piece& w = walls[i];
        Wall wall;
        wall.id = "wall-" + std::to_string(i + 1);
        wall.start = w.axis == Axis::X ? Eigen::Vector2d{w.center, w.from}
                                       : Eigen::Vector2d{w.from, w.center};
        wall.end =
            w.axis == Axis::X ? Eigen::Vector2d{w.center, w.to} : Eigen::Vector2d{w.to, w.center};
        wall.thickness = w.thickness;
        out.walls.push_back(wall);
    }

    // band points near a wall body count for that wall; the rest inside a room are furniture
    for (const LidarPoint& pt : aligned) {
        const double h = height_above(floor, pt);
        if (h < band_lo || h >= band_hi) continue;
        std::size_t nearest = walls.size();
        double nearest_d = p.object_clearance + kEps;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            const double d =
                distance_from_point_to_wall(pt.position.x(), pt.position.y(), walls[i]);
            if (d <= nearest_d) {
                nearest = i;
                nearest_d = d;
            }
        }
        if (nearest < walls.size()) {
            ++out.walls[nearest].num_points;
        } else if (is_inside_any_outline(pt.position.x(), pt.position.y(), room_outlines,
                                         p.object_clearance)) {
            out.furniture_points.push_back(pt);
        }
    }
    return out;
}

}  // namespace vcortex
