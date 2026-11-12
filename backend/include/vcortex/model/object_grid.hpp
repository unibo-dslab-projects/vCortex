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

std::vector<std::uint32_t> label_objects(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask);

std::vector<std::uint32_t> assign_non_labelled_objects(const ObjectGrid& grid, const std::vector<std::uint32_t>& mask, const std::vector<std::uint8_t>& occupied);

/// Split each piece where the tops of two neighbouring cells differ by more than height_step.
std::vector<std::uint32_t> split_by_height(const ObjectGrid& grid,
                                           const std::vector<std::uint32_t>& labels,
                                           double height_step, int min_piece_cells);

// a point where grid cells meet: corner (x, y) sits at origin + (x, y) * cell
struct GridCorner {
    int x = 0, y = 0;
};

/// Cell corners of the outer ring of one object, counter-clockwise, corners only. Holes are
/// left out, so a desk seen only at its rim comes back whole.
std::vector<GridCorner> trace_outer_ring(const ObjectGrid& grid,
                                         const std::vector<std::uint32_t>& labels,
                                         std::uint32_t object_id);

/// Straighten stair steps shorter than min_step_cells: the short edge goes and its two
/// neighbours join on one line. Spikes and notches (neighbours running opposite ways) stay.
void drop_small_segments(std::vector<GridCorner>& ring, int min_step_cells);

}  // namespace vcortex
