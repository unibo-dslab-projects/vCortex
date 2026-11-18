#include <doctest/doctest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "vcortex/model/object/boxes.hpp"

namespace vcortex {
namespace {

struct DrawnPiece {
    ObjectGrid grid;
    std::vector<std::uint32_t> labels;
};

// '#' is a cell of piece 1, a digit a cell of that piece; the first drawn row is the top one
DrawnPiece draw_piece(const std::vector<std::string>& rows) {
    DrawnPiece drawn;
    drawn.grid.cell = 0.05;
    drawn.grid.height = static_cast<int>(rows.size());
    drawn.grid.width = static_cast<int>(rows.front().size());
    drawn.labels.assign(static_cast<std::size_t>(drawn.grid.width * drawn.grid.height), 0);
    for (int row = 0; row < drawn.grid.height; ++row) {
        const std::string& line = rows[static_cast<std::size_t>(drawn.grid.height - 1 - row)];
        for (int column = 0; column < drawn.grid.width; ++column) {
            const char drawn_cell = line[static_cast<std::size_t>(column)];
            std::uint32_t& label =
                drawn.labels[static_cast<std::size_t>(row * drawn.grid.width + column)];
            if (drawn_cell == '#') label = 1;
            if (drawn_cell >= '1' && drawn_cell <= '9')
                label = static_cast<std::uint32_t>(drawn_cell - '0');
        }
    }
    return drawn;
}

// box sizes in cells, smallest first, so a test does not depend on the order boxes are found
std::vector<std::tuple<int, int>> box_sizes(const BoxPieces& pieces) {
    std::vector<std::tuple<int, int>> sizes;
    for (const Eigen::AlignedBox2i& box : pieces.boxes) {
        sizes.emplace_back(box.sizes().x() + 1, box.sizes().y() + 1);
    }
    std::sort(sizes.begin(), sizes.end());
    return sizes;
}

std::uint32_t label_at(const DrawnPiece& drawn, const BoxPieces& pieces, int column, int top_row) {
    const int row = drawn.grid.height - 1 - top_row;
    return pieces.labels[static_cast<std::size_t>(row * drawn.grid.width + column)];
}

TEST_CASE("a rectangle with a one-cell step stays one box and the step joins it") {
    const DrawnPiece drawn = draw_piece({
        "..........",
        ".########.",
        ".#########",
        ".########.",
        ".########.",
        "..........",
    });
    const BoxPieces pieces = split_into_boxes(drawn.grid, drawn.labels, 2);

    CHECK(box_sizes(pieces) == std::vector<std::tuple<int, int>>{{8, 4}});
    CHECK(label_at(drawn, pieces, 9, 2) == 1);
}

TEST_CASE("the largest rectangle goes first, not the tallest or the widest") {
    // columns 2, 3 and 1 cells high: 2 x 2 = 4 beats the 1 x 3 column and the 3 x 1 row
    const DrawnPiece drawn = draw_piece({
        ".#.",
        "##.",
        "###",
    });
    const BoxPieces pieces = split_into_boxes(drawn.grid, drawn.labels, 1);

    CHECK(box_sizes(pieces) == std::vector<std::tuple<int, int>>{{1, 1}, {1, 1}, {2, 2}});
}

TEST_CASE("a step joins a box of its own piece, not a touching box of another piece") {
    // the step of piece 1 at column 3 sits on top of piece 2's box
    const DrawnPiece drawn = draw_piece({
        "111...",
        "111...",
        "1111..",
        "..2222",
        "..2222",
    });
    const BoxPieces pieces = split_into_boxes(drawn.grid, drawn.labels, 2);

    CHECK(label_at(drawn, pieces, 3, 2) == label_at(drawn, pieces, 0, 0));
}

TEST_CASE("an L gives two boxes") {
    const DrawnPiece drawn = draw_piece({
        "##......",
        "##......",
        "##......",
        "########",
        "########",
    });
    const BoxPieces pieces = split_into_boxes(drawn.grid, drawn.labels, 2);

    CHECK(box_sizes(pieces) == std::vector<std::tuple<int, int>>{{2, 3}, {8, 2}});
}

TEST_CASE("a U gives three boxes") {
    const DrawnPiece drawn = draw_piece({
        "##....##",
        "##....##",
        "##....##",
        "########",
        "########",
    });
    const BoxPieces pieces = split_into_boxes(drawn.grid, drawn.labels, 2);

    CHECK(box_sizes(pieces) == std::vector<std::tuple<int, int>>{{2, 3}, {2, 3}, {8, 2}});
}

TEST_CASE("a ring gives four boxes and its hole stays empty") {
    const DrawnPiece drawn = draw_piece({
        "##########",
        "##########",
        "##......##",
        "##......##",
        "##......##",
        "##......##",
        "##......##",
        "##......##",
        "##########",
        "##########",
    });
    const BoxPieces pieces = split_into_boxes(drawn.grid, drawn.labels, 2);

    CHECK(pieces.boxes.size() == 4);
    CHECK(label_at(drawn, pieces, 5, 5) == 0);
}

}  // namespace
}  // namespace vcortex
