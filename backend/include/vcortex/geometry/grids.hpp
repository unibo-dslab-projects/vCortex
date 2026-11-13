#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "vcortex/model/grid.hpp"
#include "vcortex/model/object/object_grid.hpp"

namespace vcortex {

/// Check if the given coordinates are within the grid bounds
bool is_in_grid(const Grid& grid, int x, int y);
/// Returns the index of the cell at the given coordinates
std::size_t at(const Grid& grid, int x, int y);
/// Takes a point and populate the ix/iy cell coordinates, returning if the point is within the grid
bool cell_of(const Grid& g, double x, double y, int& ix, int& iy);
/// Returns the coordinates covered by a disk of the given radius, centered at the origin
std::vector<Eigen::Vector2i> disk(double radius_cells);
/// Grow the marked cells by the given offsets, returning the new marked cells
std::vector<std::uint8_t> grow(const std::vector<std::uint8_t>& marked_cells, const Grid& grid,
                               const std::vector<Eigen::Vector2i>& offsets);
/// Invert the marked cells
std::vector<std::uint8_t> invert(std::vector<std::uint8_t> marked_cells);
/// Close holes in the grid by a growing and shrinking process
std::vector<std::uint8_t> close_holes(const std::vector<std::uint8_t>& marked_cells,
                                      const Grid& grid);
/// Cast a single coordinate to an index of the grid
int cell_index(double v, double origin, double cell, int size);
/// Returns the number of cells in a given length
int num_cells_in_len(double length, double cell);
/// Returns true if the given cell has a neighbour with the given value
bool has_neighbour_with_value(const ObjectGrid& grid, const std::vector<std::uint8_t>& marked_cells,
                              int row, int column, int radius_cells, std::uint8_t value);
/// Returns the indices of the neighbour cells of the given cell, it returns only horizontal/vertical neighbours
std::vector<std::size_t> get_neighbour_indices(const ObjectGrid& grid, std::size_t index);
/// Shoelace sum, doubled to stay in integers: positive for a counter-clockwise vertex vector
long long doubled_signed_area(const std::vector<Eigen::Vector2i>& polygon);

}  // namespace vcortex
