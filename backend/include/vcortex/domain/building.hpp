#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/domain/settings.hpp"

namespace vcortex {

// mirrors schema/twin.schema.json; ifc (open bim standard) names, so an ifc export is a
// rename, not a remodel

// IfcWall; start/end are the centreline, as in ifc; the model sees only the interior face
// and shifts it out by half the thickness
struct Wall {
    std::string id;
    Vec2 start, end;
    double thickness = 0.0;
    std::size_t points = 0;
    // share of length where points were seen, rest filled in (behind furniture, gaps);
    // measurable, unlike a "confidence" score
    double coverage = 0.0;
};

// IfcSpace.
struct Room {
    std::string id;
    std::optional<std::string> name;
    std::vector<Vec2> outline;
    std::vector<std::string> wall_ids;
    bool closed = false;  // false: the free space leaked out through a missing wall
};

// IfcBuildingElementProxy: ifc's type for an element of unknown kind
struct Object {
    std::string id;
    std::string kind = "unknown";  // TODO: classify objects; nothing sets another kind yet
    std::optional<std::string> room_id, against_wall_id;
    Vec2 center, size;
    double yaw = 0.0;
    double bottom_z = 0.0;
    // lidar rarely sees tall furniture tops, so the real top may be higher; reported as seen
    double top_z_seen = 0.0;
    std::size_t points = 0;
};

struct Level {
    std::string id;
    double floor_z = 0.0;
    double wall_height = 0.0;
    HeightSource wall_height_source = HeightSource::Default;
    std::vector<Wall> walls;
    std::vector<Room> rooms;
    std::vector<Object> objects;
};

struct Building {
    std::uint32_t run_id = 0;
    std::uint32_t seq = 0;
    std::string frame_id;  // copied from Snapshot: coordinates mean nothing without their frame
    std::vector<Level> levels;
};

}  // namespace vcortex
