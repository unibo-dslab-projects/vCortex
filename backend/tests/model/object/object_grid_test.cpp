#include <doctest/doctest.h>

#include <stdexcept>
#include <vector>

#include "vcortex/model/object/grid.hpp"

namespace vcortex {
namespace {

LidarPoint point_at(float x, float y) {
    LidarPoint point;
    point.position = {x, y, 0.5F};
    return point;
}

TEST_CASE("points 50 m apart fit in the object grid") {
    const ObjectGrid grid =
        build_object_grid({point_at(0, 0), point_at(50, 50)}, Plane{}, 0.05, 3, 1);

    CHECK(grid.width > 1000);
}

TEST_CASE("points 150 m apart are too wide for the object grid") {
    CHECK_THROWS_AS(build_object_grid({point_at(0, 0), point_at(150, 150)}, Plane{}, 0.05, 3, 1),
                    std::runtime_error);
}

}  // namespace
}  // namespace vcortex
