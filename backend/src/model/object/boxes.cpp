#include "vcortex/model/object/boxes.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "vcortex/geometry/grids.hpp"
#include "vcortex/model/object/labels.hpp"

namespace vcortex {

namespace {

/// Largest rectangle whose cells are all set in is_unboxed; an empty box when none is.
/// Row by row, each column holds how many unboxed cells stack up to that row, and a stack of
/// rising heights closes, on every drop, the widest rectangle of the height that ends.
// shortcut: scans the whole grid for every box; bound it to the piece if many pieces slow it
Eigen::AlignedBox2i find_largest_full_rectangle(const ObjectGrid& grid,
                                                const std::vector<std::uint8_t>& is_unboxed) {
    // one extra column of height 0 closes every rectangle still open at the row end
    std::vector<int> unboxed_cells_stacked_up_to_row(static_cast<std::size_t>(grid.width) + 1, 0);
    const auto bar_of = [&](int column) -> int& {
        return unboxed_cells_stacked_up_to_row[static_cast<std::size_t>(column)];
    };
    Eigen::AlignedBox2i largest;
    int largest_area = 0;
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            bar_of(column) = is_unboxed[at(grid, column, row)] ? bar_of(column) + 1 : 0;
        }
        // columns whose rectangle can still grow right; their bars rise from bottom to top
        std::vector<int> open_columns;
        for (int column = 0; column <= grid.width; ++column) {
            while (!open_columns.empty() && bar_of(open_columns.back()) >= bar_of(column)) {
                const int rectangle_height = bar_of(open_columns.back());
                open_columns.pop_back();
                // the columns between the one left on the stack and this one were taller: popped
                const int first_column = open_columns.empty() ? 0 : open_columns.back() + 1;
                const int area = rectangle_height * (column - first_column);
                if (area > largest_area) {
                    largest_area = area;
                    largest = Eigen::AlignedBox2i(
                        Eigen::Vector2i(first_column, row - rectangle_height + 1),
                        Eigen::Vector2i(column - 1, row));
                }
            }
            open_columns.push_back(column);
        }
    }
    return largest;
}

bool is_too_thin(const Eigen::AlignedBox2i& box, int min_side_cells) {
    return box.isEmpty() || box.sizes().minCoeff() + 1 < min_side_cells;
}

}  // namespace

BoxPieces split_into_boxes(const ObjectGrid& grid, const std::vector<std::uint32_t>& labels,
                           int min_side_cells) {
    BoxPieces pieces;
    pieces.labels.assign(labels.size(), 0);
    if (labels.empty()) return pieces;

    const std::uint32_t piece_count = *std::max_element(labels.begin(), labels.end());
    for (std::uint32_t piece = 1; piece <= piece_count; ++piece) {
        std::vector<std::uint8_t> is_unboxed(labels.size(), 0);
        for (std::size_t index = 0; index < labels.size(); ++index) {
            is_unboxed[index] = labels[index] == piece;
        }
        for (Eigen::AlignedBox2i box = find_largest_full_rectangle(grid, is_unboxed);
             !is_too_thin(box, min_side_cells);
             box = find_largest_full_rectangle(grid, is_unboxed)) {
            pieces.boxes.push_back(box);
            const auto box_label = static_cast<std::uint32_t>(pieces.boxes.size());
            for (int row = box.min().y(); row <= box.max().y(); ++row) {
                for (int column = box.min().x(); column <= box.max().x(); ++column) {
                    pieces.labels[at(grid, column, row)] = box_label;
                    is_unboxed[at(grid, column, row)] = 0;
                }
            }
        }
    }

    // the pieces are the regions, so a leftover cell only joins a box of its own piece
    pieces.labels = assign_non_labelled_objects(grid, pieces.labels, labels);

    // labels past the boxes found are pieces too thin for one: their box bounds their cells
    const std::size_t found_box_count = pieces.boxes.size();
    pieces.boxes.resize(*std::max_element(pieces.labels.begin(), pieces.labels.end()));
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const std::uint32_t label = pieces.labels[at(grid, column, row)];
            if (label > found_box_count)
                pieces.boxes[label - 1].extend(Eigen::Vector2i(column, row));
        }
    }
    return pieces;
}

}  // namespace vcortex
