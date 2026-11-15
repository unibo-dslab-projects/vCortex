#pragma once

#include <filesystem>

#include "vcortex/domain/snapshot.hpp"

namespace vcortex {

/// Struct representing the offline input, including the map and trajectory
struct OfflineInput {
    Snapshot snapshot;
    Trajectory trajectory;
};

/// Loads the map from the given PCD file, and the trajectory from trajectory.tum beside it
/// if present, throwing on any error.
OfflineInput load_offline(const std::filesystem::path& pcd_file);

}  // namespace vcortex
