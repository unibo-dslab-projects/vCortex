#include <doctest/doctest.h>

#include <optional>
#include <vector>

#include "vcortex/model/wall/outline_detail.hpp"

namespace vcortex {
namespace {

constexpr double kStripMetres = 0.25;
constexpr Direction kEast{Axis::X, 1};
constexpr Direction kNorth{Axis::Y, 1};

// one sample per face, strips of kStripMetres from 0 along the wall; nullopt = no wall seen
std::vector<detail::WallFaceSample> samples_along(const std::vector<std::optional<double>>& faces) {
    std::vector<detail::WallFaceSample> samples;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const double from = static_cast<double>(i) * kStripMetres;
        samples.push_back({from, from + kStripMetres, faces[i]});
    }
    return samples;
}

std::vector<std::optional<double>> repeated(double face, int count) {
    return std::vector<std::optional<double>>(static_cast<std::size_t>(count), face);
}

std::vector<std::optional<double>> joined(
    std::initializer_list<std::vector<std::optional<double>>> parts) {
    std::vector<std::optional<double>> faces;
    for (const auto& part : parts) faces.insert(faces.end(), part.begin(), part.end());
    return faces;
}

void check_side(const OutlineSide& side, double face, double from, double to) {
    CHECK(side.face == doctest::Approx(face));
    CHECK(side.from == doctest::Approx(from));
    CHECK(side.to == doctest::Approx(to));
}

TEST_CASE("an L-shaped room has two walls facing the same way") {
    const auto sides = detail::find_outline_sides(
        samples_along(joined({repeated(4.0, 6), repeated(2.0, 6)})), kEast, OutlineParams{});

    REQUIRE(sides.size() == 2);
    check_side(sides[0], 4.0, 0.0, 1.5);
    check_side(sides[1], 2.0, 1.5, 3.0);
}

TEST_CASE("a door does not split the wall it is in") {
    const auto sides = detail::find_outline_sides(
        samples_along(joined({repeated(4.0, 4),
                              {std::nullopt, std::nullopt, std::nullopt, std::nullopt},
                              repeated(4.0, 4)})),
        kEast, OutlineParams{});

    REQUIRE(sides.size() == 1);
    check_side(sides[0], 4.0, 0.0, 3.0);
}

TEST_CASE("a short thing against the wall is covered by the wall and does not move it") {
    // 0.75 m long, 0.30 m in front of the wall
    const auto sides = detail::find_outline_sides(
        samples_along(joined({repeated(4.0, 4), repeated(3.7, 3), repeated(4.0, 4)})), kEast,
        OutlineParams{});

    REQUIRE(sides.size() == 1);
    check_side(sides[0], 4.0, 0.0, 2.75);
}

TEST_CASE("a thing longer than dent_length against the wall stays a wall of its own") {
    // 2.5 m long, over the 2.0 m dent_length
    const auto sides = detail::find_outline_sides(
        samples_along(joined({repeated(4.0, 4), repeated(3.7, 10), repeated(4.0, 4)})), kEast,
        OutlineParams{});

    CHECK(sides.size() == 3);
}

TEST_CASE("two walls that stop short of their corner are stretched to meet") {
    std::vector<OutlineSide> sides = {
        {kEast, 4.0, 0.3, 2.7},
        {kNorth, 3.0, 0.2, 3.6},
    };
    detail::join_outline_corners(sides, OutlineParams{});

    CHECK(sides[0].to == doctest::Approx(3.0));
    CHECK(sides[1].to == doctest::Approx(4.0));
    // no wall at the other ends: they stay where they were seen
    CHECK(sides[0].from == doctest::Approx(0.3));
    CHECK(sides[1].from == doctest::Approx(0.2));
}

TEST_CASE("a wall that pokes past its corner by less than corner_overshoot is trimmed back") {
    std::vector<OutlineSide> sides = {
        {kEast, 4.0, 0.3, 3.4},
        {kNorth, 3.0, 0.2, 3.6},
    };
    detail::join_outline_corners(sides, OutlineParams{});

    CHECK(sides[0].to == doctest::Approx(3.0));
}

TEST_CASE("a perpendicular wall that runs on far past the side is not its corner") {
    // the north wall goes on 2 m past the east wall's face: a T, not a corner
    std::vector<OutlineSide> sides = {
        {kEast, 4.0, 0.3, 1.7},
        {kNorth, 2.0, 0.2, 6.0},
    };
    detail::join_outline_corners(sides, OutlineParams{});

    CHECK(sides[0].to == doctest::Approx(1.7));
}

}  // namespace
}  // namespace vcortex
