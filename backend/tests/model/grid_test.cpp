#include <doctest/doctest.h>

#include "floor_drawing.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex {
namespace {

using test::FloorDrawing;
using test::read_floor_drawing;

// a 2.0 x 1.0 m room: 40 x 20 cells of 5 cm
FloorDrawing small_room() {
    return read_floor_drawing({
        "####################",
        "#........P.........#",
        "#..................#",
        "#..................#",
        "#..................#",
        "#..................#",
        "#..................#",
        "#..................#",
        "#..................#",
        "####################",
    });
}

TEST_CASE("the grid covers the wall band of the room") {
    const Grid grid = small_room().grid;

    CHECK(grid.width == 40);
    CHECK(grid.height == 20);
}

TEST_CASE("a stray floor point far away does not grow the grid") {
    FloorDrawing drawing = small_room();
    LidarPoint stray_floor_point;
    stray_floor_point.position = {150.0F, 0.5F, 0.0F};  // on the floor, 150 m away
    drawing.points.push_back(stray_floor_point);

    const Grid grid = build_grid(drawing.points, test::kFlatFloor, test::detail::kBandLowMetres,
                                 test::detail::kBandHighMetres, test::detail::kGridCellMetres);

    CHECK(grid.width == 40);
    CHECK(grid.height == 20);
}

}  // namespace
}  // namespace vcortex
