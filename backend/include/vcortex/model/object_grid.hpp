#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/model/floor.hpp"

namespace vcortex {

struct ObjectGrid {
    double cell = 0.0;
    Vec2 origin;
    int width = 0, height = 0;
    std::vector<std::uint8_t> is_occupied;
    std::vector<std::uint32_t> point_count;
    std::vector<float> top_metres;
    // the cell of every input point, same order as the input, so pieces map back to points
    std::vector<std::size_t> cell_of_point;
};

/// Build the map of furniture points onto a grid, for object detection.
ObjectGrid build_object_grid(const std::vector<Point>& furniture_points, const Plane& floor,
    double cell, int padding_cells, int cell_min_points);

std::vector<std::uint8_t> grow_by_cell(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                                 int radius_cells);
std::vector<std::uint8_t> shrink_by_cell(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                                int radius_cells);

}  // namespace vcortex
