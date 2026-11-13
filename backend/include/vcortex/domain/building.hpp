#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "vcortex/domain/settings.hpp"

namespace vcortex {

// mirrors schema/twin.schema.json; ifc (open bim standard) names

// IfcWall
struct Wall {
    std::string id;
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d end = Eigen::Vector2d::Zero();
    double thickness = 0.0;
    std::size_t num_points = 0;
    double coverage = 0.0;
};

// IfcSpace
struct Room {
    std::string id;
    std::optional<std::string> name;
    std::vector<Eigen::Vector2d> outline;
    std::vector<std::string> wall_ids;
    bool closed = false;
};

// IfcBuildingElementProxy
struct Object {
    std::string id;
    std::string kind = "unknown";
    std::optional<std::string> room_id, against_wall_id;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::Vector2d size = Eigen::Vector2d::Zero();
    double yaw = 0.0;
    std::vector<Eigen::Vector2d> footprint;
    double bottom_z = 0.0;
    double top_z_seen = 0.0;
    std::size_t num_points = 0;
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
    std::string frame_id;
    std::vector<Level> levels;
};

}  // namespace vcortex
