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

// one room's east faces, 0.10 m thick walls: each centerline is 5 cm behind its face
WallsAndFurniture walls_behind(const std::vector<OutlineSide>& east_faces) {
    WallParams parameters;
    parameters.thickness = 0.10;
    return find_walls({east_faces}, {}, kFlatFloor, 0.3, 2.6, parameters);
}

TEST_CASE("pieces each near the next do not chain into one wall") {
    // centerlines 1.75 (2 m long), 1.84 and 1.93: the last is 18 cm from the longest
    const WallsAndFurniture result =
        walls_behind({{kEast, 1.70, 0.0, 2.0}, {kEast, 1.79, 2.0, 3.0}, {kEast, 1.88, 3.0, 4.0}});

    REQUIRE(result.walls.size() == 2);
    CHECK(result.walls[0].start.x() == doctest::Approx((1.75 * 2.0 + 1.84) / 3.0));
    CHECK(result.walls[1].start.x() == doctest::Approx(1.93));
}

TEST_CASE("a short stray piece at the end of a line does not split the wall") {
    // centerlines 1.71 (0.3 m), 1.76, 1.80 (2 m, the longest) and 1.84: all within 10 cm of 1.80,
    // though 1.71 and 1.84 are 13 cm apart
    const WallsAndFurniture result = walls_behind({{kEast, 1.66, 0.0, 0.3},
                                                   {kEast, 1.71, 0.3, 1.3},
                                                   {kEast, 1.75, 1.3, 3.3},
                                                   {kEast, 1.79, 3.3, 4.3}});

    REQUIRE(result.walls.size() == 1);
    CHECK(result.walls[0].start.x() ==
          doctest::Approx((1.71 * 0.3 + 1.76 + 1.80 * 2.0 + 1.84) / 4.3));
}

}  // namespace
}  // namespace vcortex
