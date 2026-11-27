#include <doctest/doctest.h>

#include <vector>

#include "vcortex/model/wall/walls.hpp"

namespace vcortex {
namespace {

constexpr Direction kEast{Axis::X, 1};
constexpr Direction kWest{Axis::X, -1};
constexpr Direction kNorth{Axis::Y, 1};
constexpr Direction kSouth{Axis::Y, -1};

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

TEST_CASE("a corridor face paired with one of two rooms makes one wall, not two") {
    // corridor east face at x = 5.0 pairs with room B (y 0-5); room C (y 6-10) is left alone,
    // its 0.10 m piece centered at 5.15 lies on the same line as the paired one at 5.10
    const std::vector<std::vector<OutlineSide>> room_outlines = {
        {{kEast, 5.0, 0.0, 10.0}},
        {{kWest, 5.2, 0.0, 5.0}},
        {{kWest, 5.2, 6.0, 10.0}},
    };
    WallParams parameters;
    parameters.thickness = 0.10;
    const WallsAndFurniture result =
        find_walls(room_outlines, {}, kFlatFloor, 0.3, 2.6, parameters);

    REQUIRE(result.walls.size() == 1);
    CHECK(result.walls[0].start.x() == doctest::Approx((5.1 * 10.0 + 5.15 * 4.0) / 14.0));
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

// one room's faces, 0.10 m thick walls: each centerline is 5 cm behind its face
WallsAndFurniture walls_behind(const std::vector<OutlineSide>& faces) {
    WallParams parameters;
    parameters.thickness = 0.10;
    return find_walls({faces}, {}, kFlatFloor, 0.3, 2.6, parameters);
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

TEST_CASE("a short wall near a corner keeps its length") {
    // wall at x = 1.75 from y = 0 to 0.6; a perpendicular wall's centerline at y = 0.45 is within
    // corner_gap of both ends, but only the near end, to, snaps to it
    const WallsAndFurniture result =
        walls_behind({{kEast, 1.70, 0.0, 0.6}, {kNorth, 0.40, 0.0, 1.70}});

    REQUIRE(result.walls.size() == 2);
    const Wall& short_wall =
        result.walls[0].start.x() == doctest::Approx(1.75) ? result.walls[0] : result.walls[1];
    CHECK(short_wall.start.y() == doctest::Approx(0.0));
    CHECK(short_wall.end.y() == doctest::Approx(0.45));
}

TEST_CASE("a point diagonally past a room corner is not furniture") {
    // room 0..10 m on both axes; (10.5, 10.5) has no side facing it, yet lies outside
    const std::vector<OutlineSide> faces = {{kEast, 10.0, 0.0, 10.0},
                                            {kWest, 0.0, 0.0, 10.0},
                                            {kNorth, 10.0, 0.0, 10.0},
                                            {kSouth, 0.0, 0.0, 10.0}};
    const std::vector<LidarPoint> aligned = {{{5.0F, 5.0F, 1.0F}}, {{10.5F, 10.5F, 1.0F}}};
    const WallsAndFurniture result =
        find_walls({faces}, aligned, kFlatFloor, 0.3, 2.6, WallParams{});

    REQUIRE(result.furniture_points.size() == 1);
    CHECK(result.furniture_points[0].position.x() == doctest::Approx(5.0));
}

}  // namespace
}  // namespace vcortex
