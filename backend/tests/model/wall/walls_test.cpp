#include <doctest/doctest.h>

#include <vector>

#include "vcortex/model/wall/walls.hpp"

namespace vcortex {
namespace {

constexpr Direction kEast{Axis::X, 1};
constexpr Direction kWest{Axis::X, -1};

const Plane kFlatFloor{0.0, 0.0, 0.0};

TEST_CASE("the faces of a wall seen from two rooms make one wall as thick as their distance") {
    // left room's east face at x = 1.70, right room's west face at x = 1.82
    const std::vector<std::vector<OutlineSide>> room_outlines = {
        {{kEast, 1.70, 0.0, 3.0}},
        {{kWest, 1.82, 0.0, 3.0}},
    };
    const WallsAndFurniture result =
        find_walls(room_outlines, {}, kFlatFloor, 0.3, 2.6, WallParams{});

    REQUIRE(result.walls.size() == 1);
    CHECK(result.walls[0].start.x() == doctest::Approx(1.76));
    CHECK(result.walls[0].thickness == doctest::Approx(0.12));
}

TEST_CASE("a face seen from one room only makes a wall of default thickness behind it") {
    const std::vector<std::vector<OutlineSide>> room_outlines = {{{kEast, 1.70, 0.0, 3.0}}};
    WallParams parameters;
    parameters.thickness = 0.10;
    const WallsAndFurniture result =
        find_walls(room_outlines, {}, kFlatFloor, 0.3, 2.6, parameters);

    REQUIRE(result.walls.size() == 1);
    CHECK(result.walls[0].start.x() == doctest::Approx(1.75));
    CHECK(result.walls[0].thickness == doctest::Approx(0.10));
}

}  // namespace
}  // namespace vcortex
