#include <doctest/doctest.h>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "floor_drawing.hpp"
#include "vcortex/model/floor/free_space.hpp"
#include "vcortex/model/wall/outline.hpp"

namespace vcortex {
namespace {

using test::FloorDrawing;
using test::read_floor_drawing;

std::optional<OutlineSide> side_facing(const std::vector<OutlineSide>& outline, Axis axis,
                                       int outward) {
    for (const OutlineSide& side : outline) {
        if (side.direction.axis == axis && side.direction.outward == outward) return side;
    }
    return std::nullopt;
}

TEST_CASE("each room gets its own outline, and the wall between them is in both") {
    // 1.6 x 1.8 m rooms, 0.9 m door in the wall between them, whose middle is at x = 1.75 m
    const FloorDrawing drawing = read_floor_drawing({
        "###################################", "#................#................#",
        "#................#................#", "#................#................#",
        "#................#................#", "#................#................#",
        "#.................................#", "#.................................#",
        "#.................................#", "#.................................#",
        "#........P................P.......#", "#.................................#",
        "#.................................#", "#.................................#",
        "#.................................#", "#................#................#",
        "#................#................#", "#................#................#",
        "#................#................#", "###################################",
    });
    const FreeSpace space = find_floor_rooms(drawing.points, test::kFlatFloor, drawing.grid,
                                             drawing.poses, FreeSpaceParams{});
    const std::vector<std::vector<OutlineSide>> outlines =
        find_room_outlines(space, drawing.grid, OutlineParams{});

    REQUIRE(outlines.size() == 2);
    CHECK(outlines[0].size() == 4);
    CHECK(outlines[1].size() == 4);
    const std::uint32_t left_room = space.room_of_cell[drawing.cell_under(10, 9)];
    const std::vector<OutlineSide>& left = outlines[left_room - 1];
    const std::vector<OutlineSide>& right = outlines[left_room == 1 ? 1 : 0];
    const std::optional<OutlineSide> left_east = side_facing(left, Axis::X, 1);
    const std::optional<OutlineSide> right_west = side_facing(right, Axis::X, -1);
    REQUIRE(left_east);
    REQUIRE(right_west);
    CHECK(std::abs(left_east->face - 1.75) < 0.05);
    CHECK(std::abs(right_west->face - 1.75) < 0.05);
}

// a 1.9 m wide room with walls on three sides and its floor seen up to the top of the map:
// no wall up there, so no corner moves the east wall's end, and it stops where the strips stop
std::optional<OutlineSide> east_side_of_open_top_room(int floor_rows) {
    std::vector<std::string> rows(static_cast<std::size_t>(floor_rows),
                                  "#" + std::string(18, '.') + "#");
    rows[static_cast<std::size_t>(floor_rows / 2)][9] = 'P';
    rows.push_back(std::string(20, '#'));
    const FloorDrawing drawing = read_floor_drawing(rows);
    const FreeSpace space = find_floor_rooms(drawing.points, test::kFlatFloor, drawing.grid,
                                             drawing.poses, FreeSpaceParams{});
    const std::vector<std::vector<OutlineSide>> outlines =
        find_room_outlines(space, drawing.grid, OutlineParams{});
    REQUIRE(outlines.size() == 1);
    return side_facing(outlines[0], Axis::X, 1);
}

TEST_CASE("a wall runs to the end of the map when the map is a whole number of strips long") {
    // 19 floor rows and the south wall, 10 cm each: 40 cells of 5 cm, 8 strips of 0.25 m
    const std::optional<OutlineSide> east = east_side_of_open_top_room(19);

    REQUIRE(east);
    CHECK(east->to == doctest::Approx(2.0).epsilon(0.01));
}

TEST_CASE("a wall runs to the end of the map when the map is not a whole number of strips long") {
    // 21 floor rows and the south wall, 10 cm each: 44 cells of 5 cm, 8 strips of 0.25 m and 4
    // cells (0.20 m) over
    const std::optional<OutlineSide> east = east_side_of_open_top_room(21);

    REQUIRE(east);
    CHECK(east->to == doctest::Approx(2.2).epsilon(0.01));
}

}  // namespace
}  // namespace vcortex
