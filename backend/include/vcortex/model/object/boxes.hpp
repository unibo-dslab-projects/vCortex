#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "vcortex/model/object/grid.hpp"

namespace vcortex {

/// Pieces cut into boxes: a label per cell and the box each label stands for.
struct BoxPieces {
    std::vector<std::uint32_t> labels;  // per cell, counting from 1; 0 means no piece
    std::vector<Eigen::AlignedBox2i>
        boxes;  // boxes[label - 1]; cell columns and rows, max included
};

/// Cuts every labelled piece into boxes, so an L gives 2, a U 3 and a ring 4.
/// Takes the largest full rectangle of the piece, again and again, while both its sides are at
/// least min_side_cells; the cells left over (steps of a ragged edge) join the nearest box.
/// A piece too thin for any box keeps a label, and its box is the bounding box of its cells.
BoxPieces split_into_boxes(const ObjectGrid& grid, const std::vector<std::uint32_t>& labels,
                           int min_side_cells);

}  // namespace vcortex
