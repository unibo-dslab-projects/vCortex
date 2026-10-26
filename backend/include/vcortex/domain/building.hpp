#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/domain/settings.hpp"

namespace vcortex {

// Mirrors schema/twin.schema.json (step 12). Names follow IFC.

// IfcWall. start/end are the centreline.
struct Wall {
    std::string id;
    Vec2 start, end;
    double thickness;
    std::size_t points;
    double coverage;  // share of the length where points were seen; the rest was filled in
};

// IfcSpace.
struct Room {
    std::string id;
    std::optional<std::string> name;
    std::vector<Vec2> outline;
    std::vector<std::string> wall_ids;
    bool closed;  // false: the free space leaked out through a missing wall
};

// IfcBuildingElementProxy.
struct Object {
    std::string id;
    std::string kind;  // "unknown" for now
    std::optional<std::string> room_id, against_wall_id;
    Vec2 center, size;
    double yaw;
    double bottom_z, top_z_seen;  // the real top may be higher than the highest point seen
    std::size_t points;
};

struct Level {
    std::string id;
    double floor_z;
    double wall_height;
    HeightSource wall_height_source;
    std::vector<Wall> walls;
    std::vector<Room> rooms;
    std::vector<Object> objects;
};

struct Building {
    std::uint32_t run_id;
    std::uint32_t seq;
    std::vector<Level> levels;
};

}  // namespace vcortex
