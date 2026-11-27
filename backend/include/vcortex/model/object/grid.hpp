#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/model/floor/floor.hpp"

namespace vcortex {

struct ObjectGrid {
    double cell = 0.0;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    int width = 0, height = 0;
    std::vector<std::uint8_t> is_occupied;
    std::vector<std::uint32_t> point_count;
    std::vector<float> top_metres;
    // the cell of every input point, same order as the input, so pieces map back to points
    std::vector<std::size_t> cell_of_point;
};

/// Build the map of furniture points onto a grid, for object detection. Throws when the points
/// would need more than kMaxGridCells cells, padding included.
ObjectGrid build_object_grid(const std::vector<LidarPoint>& furniture_points, const Plane& floor,
                             double cell, int padding_cells, int cell_min_points);

/// Marks every cell that has a marked cell within radius_cells, in a square around it.
std::vector<std::uint8_t> grow_by_cell(const ObjectGrid& grid,
                                       const std::vector<std::uint8_t>& marked_cells,
                                       int radius_cells);

/// Keeps marked only the cells whose whole square of radius_cells is marked; cells past the
/// grid edge do not count.
std::vector<std::uint8_t> shrink_by_cell(const ObjectGrid& grid,
                                         const std::vector<std::uint8_t>& marked_cells,
                                         int radius_cells);

}  // namespace vcortex
