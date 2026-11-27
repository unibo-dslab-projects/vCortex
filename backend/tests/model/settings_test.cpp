#include <doctest/doctest.h>

#include <limits>
#include <string>

#include "vcortex/domain/settings.hpp"

namespace vcortex {
namespace {

TEST_CASE("the default settings pass validation") {
    CHECK_FALSE(validate(ModelSettings{}).has_value());
}

TEST_CASE("a radius is limited in cells, not metres") {
    ModelSettings settings;
    settings.objects.cell = 0.02;
    settings.objects.close_radius = 0.5;  // 25 cells: fine at the default cell, too wide here

    const auto problem = validate(settings);

    REQUIRE(problem.has_value());
    CHECK(problem->find("objects.close_radius") != std::string::npos);
}

TEST_CASE("a cell finer than the minimum is refused") {
    ModelSettings settings;
    settings.objects.cell = 0.0001;

    const auto problem = validate(settings);

    REQUIRE(problem.has_value());
    CHECK(problem->find("objects.cell") != std::string::npos);
}

TEST_CASE("nan, inf and a huge neighbour count are refused") {
    ModelSettings not_a_number;
    not_a_number.walls.thickness = std::numeric_limits<double>::quiet_NaN();
    ModelSettings infinite;
    infinite.outline.max_reach = std::numeric_limits<double>::infinity();
    ModelSettings many_neighbours;
    many_neighbours.outlier_mean_k = 500000;

    CHECK(validate(not_a_number).has_value());
    CHECK(validate(infinite).has_value());
    CHECK(validate(many_neighbours).has_value());
}

}  // namespace
}  // namespace vcortex
