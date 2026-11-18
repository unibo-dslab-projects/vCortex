#include "vcortex/model/object/objects.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>

#include "vcortex/geometry/points.hpp"
#include "vcortex/model/object/boxes.hpp"
#include "vcortex/model/object/grid.hpp"
#include "vcortex/model/object/labels.hpp"

namespace vcortex {

std::vector<Object> find_objects(const std::vector<LidarPoint>& furniture_points,
                                 const Plane& floor, const ObjectParams& parameters) {
    std::vector<Object> objects;
    if (furniture_points.empty()) return objects;

    // the margin covers the widest neighbourhood closing and opening read, plus one empty cell
    const int close_radius_cells =
        static_cast<int>(std::lround(parameters.close_radius / parameters.cell));
    const int split_radius_cells =
        static_cast<int>(std::lround(parameters.split_radius / parameters.cell));
    const int padding_cells = std::max(close_radius_cells, split_radius_cells) + 1;
    const int min_box_side_cells =
        static_cast<int>(std::lround(parameters.min_box_side / parameters.cell));
    const ObjectGrid grid = build_object_grid(furniture_points, floor, parameters.cell,
                                              padding_cells, parameters.cell_min_points);

    // closing fills the gaps inside an object, opening cuts the thin bridges between two;
    // the cut cells then go back to the nearest piece
    const std::vector<std::uint8_t> closed = shrink_by_cell(
        grid, grow_by_cell(grid, grid.is_occupied, close_radius_cells), close_radius_cells);
    const std::vector<std::uint8_t> cores =
        grow_by_cell(grid, shrink_by_cell(grid, closed, split_radius_cells), split_radius_cells);
    std::vector<std::uint32_t> labels = assign_non_labelled_objects(
        grid, label_objects(grid, cores), std::vector<std::uint32_t>(closed.begin(), closed.end()));
    if (parameters.height_step > 0) {
        labels = split_by_height(grid, labels, parameters.height_step, parameters.min_piece_cells);
    }

    const BoxPieces boxes = split_into_boxes(grid, labels, min_box_side_cells);

    // label 0 holds the points of cells under cell_min_points: noise, never an object
    const std::uint32_t piece_count = *std::max_element(labels.begin(), labels.end());
    std::vector<std::size_t> point_count_of_piece(piece_count + 1, 0);
    std::vector<std::vector<LidarPoint>> points_of_box(boxes.boxes.size() + 1);
    std::vector<std::uint32_t> piece_of_box(boxes.boxes.size() + 1, 0);
    for (std::size_t i = 0; i < furniture_points.size(); ++i) {
        const std::size_t cell_index = grid.cell_of_point[i];
        const std::uint32_t box_label = boxes.labels[cell_index];
        ++point_count_of_piece[labels[cell_index]];
        points_of_box[box_label].push_back(furniture_points[i]);
        piece_of_box[box_label] = labels[cell_index];
    }

    objects.reserve(boxes.boxes.size());
    for (std::size_t box_label = 1; box_label < points_of_box.size(); ++box_label) {
        const std::vector<LidarPoint>& points = points_of_box[box_label];
        // the whole piece must be big enough, so a small arm of a large object stays
        // a box that only closing filled has no point, so no height
        if (points.empty() || point_count_of_piece[piece_of_box[box_label]] <
                                  static_cast<std::size_t>(parameters.cluster_min_points)) {
            continue;
        }

        const Eigen::AlignedBox3f bounds = get_points_bounding_box(points);
        // a box spans its cells whole, so the far corner is one cell past its last cell
        const Eigen::AlignedBox2i& cells = boxes.boxes[box_label - 1];
        const Eigen::Vector2d near_corner = grid.origin + cells.min().cast<double>() * grid.cell;
        const Eigen::Vector2d far_corner =
            grid.origin + (cells.max() + Eigen::Vector2i::Ones()).cast<double>() * grid.cell;

        Object object;
        object.center = (near_corner + far_corner) / 2.0;
        object.size = far_corner - near_corner;
        object.bottom_z = bounds.min().z();
        object.top_z_seen = bounds.max().z();
        object.num_points = points.size();
        objects.push_back(object);
    }

    std::sort(objects.begin(), objects.end(), [](const Object& a, const Object& b) {
        return std::tie(a.center.x(), a.center.y()) < std::tie(b.center.x(), b.center.y());
    });
    for (std::size_t i = 0; i < objects.size(); ++i) {
        objects[i].id = "object-" + std::to_string(i + 1);
    }
    return objects;
}

}  // namespace vcortex
