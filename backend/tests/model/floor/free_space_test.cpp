#include <doctest/doctest.h>

#include "floor_drawing.hpp"
#include "vcortex/model/floor/free_space.hpp"

namespace vcortex {
namespace {

using test::FloorDrawing;
using test::read_floor_drawing;

// default settings, as model.json leaves them
FreeSpace find_rooms(const FloorDrawing &drawing) {
  return find_floor_rooms(drawing.points, test::kFlatFloor, drawing.grid,
                          drawing.poses, FreeSpaceParams{});
}

std::uint32_t room_under(const FreeSpace &space, const FloorDrawing &drawing,
                         int row, int column) {
  return space.room_of_cell[drawing.cell_under(row, column)];
}

// 1.6 x 1.8 m rooms, 0.9 m door in the wall between them
const std::vector<std::string> kTwoRoomsWalkedBoth = {
    "###################################",
    "#................#................#",
    "#................#................#",
    "#................#................#",
    "#................#................#",
    "#................#................#",
    "#.................................#",
    "#.................................#",
    "#.................................#",
    "#.................................#",
    "#........P................P.......#",
    "#.................................#",
    "#.................................#",
    "#.................................#",
    "#.................................#",
    "#................#................#",
    "#................#................#",
    "#................#................#",
    "#................#................#",
    "###################################",
};

TEST_CASE("a door between walls splits two rooms") {
  const FloorDrawing drawing = read_floor_drawing(kTwoRoomsWalkedBoth);
  const FreeSpace space = find_rooms(drawing);

  CHECK(space.room_count == 2);
  const std::uint32_t left = room_under(space, drawing, 10, 9);
  const std::uint32_t right = room_under(space, drawing, 10, 26);
  CHECK(left != 0);
  CHECK(right != 0);
  CHECK(left != right);
}

TEST_CASE("an aisle between desks does not split a room") {
  // 3.6 x 2.0 m room; two 0.8 x 0.6 m desks leave a 0.8 m aisle, as narrow as a
  // door, and a 1.4 x 2.0 m pocket behind it: too big to pass as noise
  const FloorDrawing drawing = read_floor_drawing({
      "######################################",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#....................................#",
      "#....................................#",
      "#....................................#",
      "#....................................#",
      "#.......P............................#",
      "#....................................#",
      "#....................................#",
      "#....................................#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "#..............dddddddd..............#",
      "######################################",
  });
  const FreeSpace space = find_rooms(drawing);

  CHECK(space.room_count == 1);
  CHECK(room_under(space, drawing, 11, 8) ==
        room_under(space, drawing, 11, 30));
}

TEST_CASE("a small room behind a door is still a room") {
  // 2.0 x 2.0 m room, 1.2 x 1.2 m closet (1.44 m2) behind a 0.8 m door
  const FloorDrawing drawing = read_floor_drawing({
      "###################################",
      "#....................#............#",
      "#....................#............#",
      "#.................................#",
      "#.................................#",
      "#.................................#",
      "#.................................#",
      "#...........................P.....#",
      "#.................................#",
      "#.................................#",
      "#.................................#",
      "#..........P.........#............#",
      "#....................#............#",
      "#....................##############",
      "#....................#",
      "#....................#",
      "#....................#",
      "#....................#",
      "#....................#",
      "#....................#",
      "#....................#",
      "######################",
  });
  const FreeSpace space = find_rooms(drawing);

  CHECK(space.room_count == 2);
  CHECK(room_under(space, drawing, 11, 11) !=
        room_under(space, drawing, 7, 29));
}

TEST_CASE("a room seen through a door but never walked joins the walked room") {
  // as the two rooms above, with no pose in the right one: it splits once walked
  std::vector<std::string> rows = kTwoRoomsWalkedBoth;
  rows[10] = "#........P........................#";
  const FloorDrawing drawing = read_floor_drawing(rows);
  const FreeSpace space = find_rooms(drawing);

  CHECK(space.room_count == 1);
  CHECK(room_under(space, drawing, 10, 9) != 0);
  CHECK(room_under(space, drawing, 10, 26) ==
        room_under(space, drawing, 10, 9));
}

} // namespace
} // namespace vcortex
