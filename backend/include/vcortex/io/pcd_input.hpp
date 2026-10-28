#pragma once

#include <filesystem>

#include "vcortex/domain/snapshot.hpp"

namespace vcortex {

// Offline mode: a saved map.pcd, plus trajectory.tum from the same folder when present, so a
// map can be studied and the model tuned without the robot.
struct OfflineInput {
    Snapshot snapshot;
    Trajectory trajectory;
};

OfflineInput load_offline(const std::filesystem::path& pcd_file);

}  // namespace vcortex
