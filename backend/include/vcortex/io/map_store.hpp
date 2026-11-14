#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "vcortex/domain/snapshot.hpp"

namespace vcortex {

// what the model worker runs on, copied out under the lock
struct MapCopy {
    Snapshot snapshot;
    Trajectory trajectory;
};

struct StoreStatus {
    std::optional<std::uint32_t> run_id;  // empty until the first map
    std::uint32_t seq = 0;
    std::size_t points = 0, poses = 0;
};

// the backend's copy of the source's map: one point per voxel, keyed like the source keys it
// one mutex for everything: take() copies out, so the model runs unlocked and ingest never
// waits for it
class MapStore {
public:
    /// Forgets the old map and trajectory, ready for a new source with the given frame and voxel size.
    void reset(std::string frame_id, double voxel_size);

    /// Stores a full map. A new run_id drops the old points and poses; the same run_id
    /// replaces each point in its voxel.
    void put_map(std::uint32_t run_id, std::uint32_t seq, const std::vector<LidarPoint>& points);

    /// Stores a delta and returns where each point landed in the point list (new voxels go at the end).
    /// Returns nothing, and stores nothing, if seq does not follow the last one of this run.
    std::optional<std::vector<std::uint32_t>> put_delta(std::uint32_t run_id, std::uint32_t seq,
                                                        const std::vector<LidarPoint>& points);

    /// Adds the poses to the trajectory, or drops them if they belong to another run.
    void put_poses(std::uint32_t run_id, const std::vector<Pose>& poses);

    /// Returns a copy of the map if it changed since the last take, nothing otherwise.
    std::optional<MapCopy> take();

    /// Returns a copy of the map without marking it as taken, for a viewer that just joined.
    std::optional<MapCopy> peek() const;

    /// Marks the map as changed, so the model runs again on it (e.g. after new settings).
    void touch();

    /// Returns the current run, seq and the number of points and poses.
    StoreStatus status() const;

private:
    struct KeyHash {
        /// Returns the hash of a voxel index.
        std::size_t operator()(const Eigen::Vector3i& k) const;
    };

    /// Returns the voxel index in x, y and z of the given point.
    Eigen::Vector3i key_of(const LidarPoint& p) const;
    /// Inserts the point, or replaces the one already in its voxel, returning its index in points_.
    std::size_t upsert(const LidarPoint& p);
    /// Returns a copy of the map; the caller must already hold mutex_.
    MapCopy copy_locked() const;

    mutable std::mutex mutex_;
    std::string frame_id_;
    double voxel_size_ = 0.05;
    std::optional<std::uint32_t> run_id_;
    std::uint32_t seq_ = 0;
    // points in a vector, not as map values: take() then copies one contiguous block
    std::vector<LidarPoint> points_;
    std::unordered_map<Eigen::Vector3i, std::size_t, KeyHash> index_;
    Trajectory trajectory_;
    bool dirty_ = false;
};

}  // namespace vcortex
