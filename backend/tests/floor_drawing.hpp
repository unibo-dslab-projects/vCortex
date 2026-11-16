#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/snapshot.hpp"
#include "vcortex/geometry/grids.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/grid.hpp"

namespace vcortex::test {

/// A floor drawn as text, turned into the points a lidar would return over it.
///
/// One character is a 10 cm square, row 0 on top, the floor flat at z = 0:
///   '.'  floor seen
///   '#'  wall: floor, low points (10-30 cm) and tall points (1.0, 1.8 m)
///   'd'  desk: floor, low points and its top (0.75 m), nothing tall
///   'P'  floor seen, with a robot pose in the middle
///   ' '  nothing seen
/// Each character gets 4 x 4 points 2.5 cm apart, so every 5 cm grid cell holds
/// 2 x 2 of them and none reads as unseen floor.
struct FloorDrawing {
  std::vector<LidarPoint> points;
  Trajectory poses;
  Grid grid; // 5 cm cells over the band the model reads walls in, as in
             // run_model
  int row_count = 0;

  /// Index of the grid cell under the middle of the character at (row, column).
  std::size_t cell_under(int row, int column) const;
};

inline const Plane kFlatFloor{};

FloorDrawing read_floor_drawing(const std::vector<std::string> &rows);

// ---

namespace detail {

constexpr double kCharacterMetres = 0.10;
constexpr int kPointsPerCharacterSide = 4;
constexpr double kGridCellMetres = 0.05;
// band of run_model with the default wall height 2.70 m: candidates.band_lo
// to 2.70 - 0.10
constexpr double kBandLowMetres = 0.30;
constexpr double kBandHighMetres = 2.60;

inline std::vector<double> heights_of(char symbol) {
  switch (symbol) {
  case '.':
  case 'P':
    return {0.0};
  case '#':
    return {0.0, 0.15, 0.25, 1.0, 1.8};
  case 'd':
    return {0.0, 0.15, 0.25, 0.75};
  default:
    return {};
  }
}

inline Eigen::Vector2d middle_of(int row, int column, int row_count) {
  return {(column + 0.5) * kCharacterMetres,
          (row_count - 1 - row + 0.5) * kCharacterMetres};
}

} // namespace detail

inline std::size_t FloorDrawing::cell_under(int row, int column) const {
  const Eigen::Vector2d middle = detail::middle_of(row, column, row_count);
  int cell_x = 0, cell_y = 0;
  cell_of(grid, middle.x(), middle.y(), cell_x, cell_y);
  return at(grid, cell_x, cell_y);
}

inline FloorDrawing read_floor_drawing(const std::vector<std::string> &rows) {
  using namespace detail;
  FloorDrawing drawing;
  drawing.row_count = static_cast<int>(rows.size());
  const double point_spacing = kCharacterMetres / kPointsPerCharacterSide;
  for (int row = 0; row < drawing.row_count; ++row) {
    const std::string &line = rows[static_cast<std::size_t>(row)];
    for (int column = 0; column < static_cast<int>(line.size()); ++column) {
      const char symbol = line[static_cast<std::size_t>(column)];
      const Eigen::Vector2d middle = middle_of(row, column, drawing.row_count);
      if (symbol == 'P') {
        Pose pose;
        pose.position = {middle.x(), middle.y(), 0.0};
        drawing.poses.push_back(pose);
      }
      const Eigen::Vector2d corner =
          middle - Eigen::Vector2d::Constant(kCharacterMetres / 2.0);
      for (const double height : heights_of(symbol)) {
        for (int i = 0; i < kPointsPerCharacterSide; ++i) {
          for (int j = 0; j < kPointsPerCharacterSide; ++j) {
            LidarPoint point;
            point.position = {
                static_cast<float>(corner.x() + (i + 0.5) * point_spacing),
                static_cast<float>(corner.y() + (j + 0.5) * point_spacing),
                static_cast<float>(height)};
            drawing.points.push_back(point);
          }
        }
      }
    }
  }
  drawing.grid = build_grid(drawing.points, kFlatFloor, kBandLowMetres,
                            kBandHighMetres, kGridCellMetres);
  return drawing;
}

} // namespace vcortex::test
