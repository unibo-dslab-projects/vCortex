#pragma once

#include <filesystem>

#include "vcortex/domain/snapshot.hpp"

namespace vcortex {

// offline mode: saved map.pcd (plus trajectory.tum beside it), to study a map and tune the
// model without the robot
struct OfflineInput {
    Snapshot snapshot;
    Trajectory trajectory;
};

OfflineInput load_offline(const std::filesystem::path& pcd_file);

}  // namespace vcortex
