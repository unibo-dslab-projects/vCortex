#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "vcortex/model/grid.hpp"
#include "vcortex/model/object/object_grid.hpp"

namespace vcortex {

using Mask = std::vector<std::uint8_t>;

bool in_grid(const Grid& grid, int x, int y);

std::size_t at(const Grid& grid, int x, int y);

bool cell_of(const Grid& g, double x, double y, int& ix, int& iy);

// cell steps from the centre cell to every cell within the radius
std::vector<Eigen::Vector2i> disk(double radius_cells);

Mask grow(const Mask& m, const Grid& g, const std::vector<Eigen::Vector2i>& offsets);

Mask invert(Mask m);

// grow, then shrink; shrink = grow the opposite, so the grid edge does not eat the mask
Mask close_holes(const Mask& m, const Grid& g);

int cell_index(double v, double origin, double cell, int size);

int cells_in(double length, double cell);

bool has_neighbour_with_value(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                              int row, int column, int radius_cells, std::uint8_t value);

// only the 4 side neighbours inside the grid, so cells touching at a corner stay apart
std::vector<std::size_t> neighbour_indices(const ObjectGrid& grid, std::size_t index);

// shoelace sum, doubled to stay in integers: positive for a counter-clockwise ring
long long doubled_signed_area(const std::vector<Eigen::Vector2i>& ring);

}  // namespace vcortex
