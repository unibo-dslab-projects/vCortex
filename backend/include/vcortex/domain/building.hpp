#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/domain/settings.hpp"

namespace vcortex {

// Mirrors schema/twin.schema.json. Names follow IFC, the open BIM standard, so a later IFC
// export is a renaming, not a remodelling.

// IfcWall. start/end are the centreline, as in IFC and on an architect's drawing; the model sees
// only the interior face and shifts it outwards by half the thickness.
struct Wall {
    std::string id;
    Vec2 start, end;
    double thickness = 0.0;
    std::size_t points = 0;
    // Share of the length where points were seen; the rest was filled in (behind furniture, or
    // across a gap). A measured share can be checked, a "confidence" score could not.
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

// IfcBuildingElementProxy: IFC's type for an element whose kind is not known.
struct Object {
    std::string id;
    std::string kind = "unknown";  // TODO: classify objects; nothing sets another kind yet
    std::optional<std::string> room_id, against_wall_id;
    Vec2 center, size;
    double yaw = 0.0;
    double bottom_z = 0.0;
    // The lidar rarely sees the top of tall furniture, so the real top may be higher. Reported as
    // seen rather than guessed.
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
    std::vector<Level> levels;
};

}  // namespace vcortex
