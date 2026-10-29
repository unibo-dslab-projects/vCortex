#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

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
    void put_map(std::uint32_t run_id, std::uint32_t seq, const std::vector<Point>& points,
                 bool has_hits, bool has_intensity);

    // false when the delta does not follow the last seq of this run: nothing is applied and
    // the source must send a fresh map
    bool put_delta(std::uint32_t run_id, std::uint32_t seq, const std::vector<Point>& points);

    // replaces hits of voxels already known; unknown voxels and other runs are ignored
    void put_hits(std::uint32_t run_id, const std::vector<Point>& points);

    // poses of another run belong to a map this store does not hold, so they are dropped
    void put_poses(std::uint32_t run_id, const std::vector<Pose>& poses);

    // the map if it changed since the last take, else nothing
    std::optional<MapCopy> take();

    // run the model again on the same map, e.g. after new settings
    void touch();

    StoreStatus status() const;

private:
    struct Key {
        std::int32_t x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const;
    };

    Key key_of(const Point& p) const;
    void upsert(const Point& p);

    mutable std::mutex mutex_;
    std::string frame_id_;
    double voxel_size_ = 0.05;
    std::optional<std::uint32_t> run_id_;
    std::uint32_t seq_ = 0;
    bool has_hits_ = false, has_intensity_ = false;
    // points in a vector, not as map values: take() then copies one contiguous block
    std::vector<Point> points_;
    std::unordered_map<Key, std::size_t, KeyHash> index_;
    Trajectory trajectory_;
    bool dirty_ = false;
};

}  // namespace vcortex
