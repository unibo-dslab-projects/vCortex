#pragma once

#include <filesystem>

#include "vcortex/domain/snapshot.hpp"

namespace vcortex {

// Step 4: map.pcd (+ trajectory.tum next to it, if present) -> Snapshot + Trajectory.
struct OfflineInput {
    Snapshot snapshot;
    Trajectory trajectory;
};

OfflineInput load_offline(const std::filesystem::path& pcd_file);

}  // namespace vcortex
