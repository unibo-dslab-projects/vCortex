#include <doctest/doctest.h>

#include <cmath>
#include <optional>

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
        find_room_outlines(space, drawing.grid, drawing.points, test::kFlatFloor,
                           test::detail::kBandHighMetres, OutlineParams{});

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

}  // namespace
}  // namespace vcortex
