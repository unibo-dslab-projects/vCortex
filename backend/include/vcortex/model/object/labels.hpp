#pragma once

#include <cstdint>
#include <vector>

#include "vcortex/model/object/grid.hpp"

namespace vcortex {

/// Gives each 4-connected group of marked cells its own label, counting from 1; 0 means
/// not marked.
std::vector<std::uint32_t> label_objects(const ObjectGrid& grid,
                                         const std::vector<std::uint8_t>& marked_cells);

/// Spreads each label to the cells it touches, nearest label first, never leaving the region of
/// the cell it spreads from; region_of_cell is 0 outside every region. Cells of a region that no
/// label reaches get new labels.
std::vector<std::uint32_t> assign_non_labelled_objects(
    const ObjectGrid& grid, const std::vector<std::uint32_t>& labels,
    const std::vector<std::uint32_t>& region_of_cell);

/// Split each piece where the tops of two neighbouring cells differ by more than height_step.
std::vector<std::uint32_t> split_by_height(const ObjectGrid& grid,
                                           const std::vector<std::uint32_t>& labels,
                                           double height_step, int min_piece_cells);

}  // namespace vcortex
