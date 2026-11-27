#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "vcortex/model/grid.hpp"
#include "vcortex/model/object/grid.hpp"

namespace vcortex {

/// Most cells one grid may hold: 2048 x 2048, a 100 m square at 5 cm. Measured on a floor of
/// desks, an object grid this size takes 2.5 s and 0.1 GB; the cost grows faster than the area.
inline constexpr std::size_t kMaxGridCells = std::size_t{1} << 22;

/// Whether cell (x, y) lies inside grid; works for Grid and ObjectGrid.
template <typename AnyGrid>
bool is_in_grid(const AnyGrid& grid, int x, int y) {
    return x >= 0 && x < grid.width && y >= 0 && y < grid.height;
}
/// Index of cell (x, y), column x and row y, in the row-major vectors of grid; works for Grid
/// and ObjectGrid. The one place the layout is written down, so x and y are never swapped.
template <typename AnyGrid>
std::size_t at(const AnyGrid& grid, int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
           static_cast<std::size_t>(x);
}
/// Takes a point and populate the ix/iy cell coordinates, returning if the point is within the grid
bool cell_of(const Grid& g, double x, double y, int& ix, int& iy);
/// Returns the coordinates covered by a disk of the given radius, centered at the origin
std::vector<Eigen::Vector2i> disk(double radius_cells);
/// Grow the marked cells by the given offsets, returning the new marked cells
std::vector<std::uint32_t> grow(const std::vector<std::uint32_t>& marked_cells, const Grid& grid,
                                const std::vector<Eigen::Vector2i>& offsets);
/// Invert the marked cells
std::vector<std::uint32_t> invert(std::vector<std::uint32_t> marked_cells);
/// Close holes in the grid by a growing and shrinking process
std::vector<std::uint32_t> close_holes(const std::vector<std::uint32_t>& marked_cells,
                                       const Grid& grid);
/// Cast a single coordinate to an index of the grid
int cell_index(double v, double origin, double cell, int size);
/// Returns the number of cells in a given length
int num_cells_in_len(double length, double cell);
/// Returns true if the given cell has a neighbour with the given value
bool has_neighbour_with_value(const ObjectGrid& grid, const std::vector<std::uint8_t>& marked_cells,
                              int row, int column, int radius_cells, std::uint8_t value);
/// Returns the indices of the neighbour cells of the given cell, it returns only
/// horizontal/vertical neighbours
std::vector<std::size_t> get_neighbour_indices(const ObjectGrid& grid, std::size_t index);
/// Shoelace sum, doubled to stay in integers: positive for a counter-clockwise vertex vector
long long doubled_signed_area(const std::vector<Eigen::Vector2i>& polygon);

}  // namespace vcortex
