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
    // new source (ingest hello, or the offline file): forgets the old map and trajectory
    void reset(std::string frame_id, double voxel_size);

    // a new run_id is a new map: old points and poses go; the same run_id is a resync, so
    // every record replaces its voxel
    void put_map(std::uint32_t run_id, std::uint32_t seq, const std::vector<LidarPoint>& points);

    // where each record landed in the map's point list (a new voxel goes at the end), so a
    // viewer holding the same list can replace points in place; nothing when the delta does
    // not follow the last seq of this run: nothing is applied and the source must send a
    // fresh map
    std::optional<std::vector<std::uint32_t>> put_delta(std::uint32_t run_id, std::uint32_t seq,
                                                        const std::vector<LidarPoint>& points);


    // poses of another run belong to a map this store does not hold, so they are dropped
    void put_poses(std::uint32_t run_id, const std::vector<Pose>& poses);

    // the map if it changed since the last take, else nothing
    std::optional<MapCopy> take();

    // the map as it is, leaving it marked changed or not; for a viewer that just joined
    std::optional<MapCopy> peek() const;

    // run the model again on the same map, e.g. after new settings
    void touch();

    StoreStatus status() const;

private:
    struct KeyHash {
        std::size_t operator()(const Eigen::Vector3i& k) const;
    };

    // voxel index in x, y and z
    Eigen::Vector3i key_of(const LidarPoint& p) const;
    // returns the voxel's index in points_
    std::size_t upsert(const LidarPoint& p);
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
