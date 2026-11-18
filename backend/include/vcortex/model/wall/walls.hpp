#pragma once

#include <vector>

#include "vcortex/domain/building.hpp"
#include "vcortex/domain/lidar_point.hpp"
#include "vcortex/domain/settings.hpp"
#include "vcortex/model/floor/floor.hpp"
#include "vcortex/model/wall/outline.hpp"

namespace vcortex {

/// What find_walls returns: the walls, and the band points that belong to no wall.
struct WallsAndFurniture {
    std::vector<Wall> walls;                   // ids "wall-1", "wall-2", ... in sorted order
    std::vector<LidarPoint> furniture_points;  // inside a room outline, far from every wall
};

/// The walls of all rooms, from their outlines (aligned frame in and out, metres).
/// Each outline side is the inner face of a wall. Two sides of different rooms facing each other
/// 5 to 35 cm apart, and overlapping, are the two faces of one wall between the rooms: its
/// centerline is halfway, its thickness their distance. A side with no partner is a wall seen
/// from one room only: p.thickness thick, its centerline half of it behind the face.
/// Walls on one line are joined across gaps up to p.min_gap, and wall ends are snapped to a
/// perpendicular wall within p.corner_gap.
/// Band points (band_lo to band_hi above floor) within p.object_clearance of a wall count for
/// it; the others inside a room outline are furniture, those past every outline are dropped.
WallsAndFurniture find_walls(const std::vector<std::vector<OutlineSide>>& room_outlines,
                             const std::vector<LidarPoint>& aligned, const Plane& floor,
                             double band_lo, double band_hi, const WallParams& p);

}  // namespace vcortex
