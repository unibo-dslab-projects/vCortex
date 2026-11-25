#include <doctest/doctest.h>

#include <cstddef>
#include <string>
#include <vector>

#include "floor_drawing.hpp"
#include "vcortex/model/model.hpp"

namespace vcortex {
namespace {

using test::FloorDrawing;
using test::read_floor_drawing;

TEST_CASE("a glass room with one desk in the middle keeps its four walls") {
    // 2.8 x 2.8 m inside: the desk is the only thing above candidates.band_lo
    std::vector<std::string> rows(28, "g" + std::string(28, '.') + "g");
    for (std::size_t row = 12; row < 16; ++row) rows[row].replace(10, 8, 8, 'd');
    rows[22][14] = 'P';
    rows.insert(rows.begin(), std::string(30, 'g'));
    rows.push_back(std::string(30, 'g'));
    const FloorDrawing drawing = read_floor_drawing(rows);
    Snapshot snapshot;
    snapshot.points = drawing.points;

    const ModelResult result = run_model(snapshot, drawing.poses, ModelSettings{});

    REQUIRE(result.building.levels.size() == 1);
    CHECK(result.building.levels[0].walls.size() == 4);
    CHECK(result.building.levels[0].rooms.size() == 1);
}

}  // namespace
}  // namespace vcortex
