#include "vcortex/model/object/objects.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "vcortex/geometry/points.hpp"
#include "vcortex/model/wall/candidates.hpp"
#include "vcortex/model/object/object_grid.hpp"

namespace vcortex {

namespace {

constexpr double kEps = 1e-9;

bool overlaps(double a0, double a1, double b0, double b1) { return a0 < b1 && b0 < a1; }

// a wall in the aligned frame as a line of constant x or y, so a box side can be measured
// against its body; walls are axis-aligned here, they turn only in the map frame
struct WallLine {
    Axis axis = Axis::X;
    double position = 0.0;        // the constant coordinate
    double from = 0.0, to = 0.0;  // extent along the wall
    double half_thickness = 0.0;
};

WallLine wall_line(const Wall& wall) {
    WallLine line;
    line.axis = wall.start.x == wall.end.x ? Axis::X : Axis::Y;
    line.position = line.axis == Axis::X ? wall.start.x : wall.start.y;
    const double along_start = line.axis == Axis::X ? wall.start.y : wall.start.x;
    const double along_end = line.axis == Axis::X ? wall.end.y : wall.end.x;
    line.from = std::min(along_start, along_end);
    line.to = std::max(along_start, along_end);
    line.half_thickness = wall.thickness / 2.0;
    return line;
}

// gap between the box range [box_from, box_to] across the wall and the wall body
double box_to_wall_distance(double box_from, double box_to, const WallLine& line) {
    const double body_from = line.position - line.half_thickness;
    const double body_to = line.position + line.half_thickness;
    return std::max({0.0, body_from - box_to, box_from - body_to});
}

struct FootprintBox {
    double min_x = 0.0, max_x = 0.0;
    double min_y = 0.0, max_y = 0.0;
};

FootprintBox footprint_box(const std::vector<Vec2>& footprint) {
    FootprintBox box{footprint.front().x, footprint.front().x, footprint.front().y,
                     footprint.front().y};
    for (const Vec2& vertex : footprint) {
        box.min_x = std::min(box.min_x, vertex.x);
        box.max_x = std::max(box.max_x, vertex.x);
        box.min_y = std::min(box.min_y, vertex.y);
        box.max_y = std::max(box.max_y, vertex.y);
    }
    return box;
}

}  // namespace

std::vector<Object> find_objects(const std::vector<Point>& furniture_points, const Plane& floor,
                                 const std::vector<Wall>& walls, const ObjectParams& parameters) {
    std::vector<Object> objects;
    if (furniture_points.empty()) return objects;

    // the margin covers the widest neighbourhood closing and opening read, plus one empty cell
    const int close_radius_cells =
        static_cast<int>(std::lround(parameters.close_radius / parameters.cell));
    const int split_radius_cells =
        static_cast<int>(std::lround(parameters.split_radius / parameters.cell));
    const int padding_cells = std::max(close_radius_cells, split_radius_cells) + 1;
    const int min_step_cells = static_cast<int>(std::lround(parameters.min_step / parameters.cell));
    const ObjectGrid grid = build_object_grid(furniture_points, floor, parameters.cell,
                                              padding_cells, parameters.cell_min_points);

    // closing fills the gaps inside an object, opening cuts the thin bridges between two;
    // the cut cells then go back to the nearest piece
    const std::vector<std::uint8_t> closed = shrink_by_cell(
        grid, grow_by_cell(grid, grid.is_occupied, close_radius_cells), close_radius_cells);
    const std::vector<std::uint8_t> cores = grow_by_cell(
        grid, shrink_by_cell(grid, closed, split_radius_cells), split_radius_cells);
    std::vector<std::uint32_t> labels =
        assign_non_labelled_objects(grid, label_objects(grid, cores), closed);
    if (parameters.height_step > 0) {
        labels = split_by_height(grid, labels, parameters.height_step, parameters.min_piece_cells);
    }

    // label 0 holds the points of cells under cell_min_points: noise, never an object
    const std::uint32_t piece_count = *std::max_element(labels.begin(), labels.end());
    std::vector<std::vector<Point>> pieces(piece_count + 1);
    for (std::size_t i = 0; i < furniture_points.size(); ++i) {
        pieces[labels[grid.cell_of_point[i]]].push_back(furniture_points[i]);
    }

    std::vector<WallLine> wall_lines;
    wall_lines.reserve(walls.size());
    for (const Wall& wall : walls) wall_lines.push_back(wall_line(wall));

    objects.reserve(piece_count);
    for (std::size_t label = 1; label < pieces.size(); ++label) {
        const std::vector<Point>& points = pieces[label];
        if (points.size() < static_cast<std::size_t>(parameters.cluster_min_points)) continue;

        const Bounds bounds = point_bounds(points);

        std::vector<GridCorner> ring =
            trace_outer_ring(grid, labels, static_cast<std::uint32_t>(label));
        drop_small_segments(ring, min_step_cells);

        Object object;
        for (const GridCorner& corner : ring) {
            object.footprint.push_back({grid.origin.x + corner.x * grid.cell,
                                        grid.origin.y + corner.y * grid.cell});
        }
        // points sit anywhere in their cell, so their box can be a cell short of what is drawn
        const FootprintBox box = footprint_box(object.footprint);
        object.center = {(box.min_x + box.max_x) / 2.0, (box.min_y + box.max_y) / 2.0};
        object.size = {box.max_x - box.min_x, box.max_y - box.min_y};
        object.bottom_z = bounds.min_z;
        object.top_z_seen = bounds.max_z;
        object.points = points.size();

        double best_distance = parameters.against_wall_distance + kEps;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            const WallLine& line = wall_lines[i];
            const bool is_along = line.axis == Axis::X
                                      ? overlaps(box.min_y, box.max_y, line.from, line.to)
                                      : overlaps(box.min_x, box.max_x, line.from, line.to);
            const double distance = line.axis == Axis::X
                                        ? box_to_wall_distance(box.min_x, box.max_x, line)
                                        : box_to_wall_distance(box.min_y, box.max_y, line);
            if (is_along && distance <= best_distance) {
                best_distance = distance;
                object.against_wall_id = walls[i].id;
            }
        }
        objects.push_back(object);
    }

    std::sort(objects.begin(), objects.end(), [](const Object& a, const Object& b) {
        return std::tie(a.center.x, a.center.y) < std::tie(b.center.x, b.center.y);
    });
    for (std::size_t i = 0; i < objects.size(); ++i) {
        objects[i].id = "object-" + std::to_string(i + 1);
    }
    return objects;
}

}  // namespace vcortex
