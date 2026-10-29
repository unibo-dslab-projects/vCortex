#include "vcortex/io/map_store.hpp"

#include <cmath>
#include <utility>

namespace vcortex {

std::size_t MapStore::KeyHash::operator()(const Key& k) const {
    // large odd primes spread neighbouring voxels across buckets
    const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.x));
    const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.y));
    const auto uz = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.z));
    return static_cast<std::size_t>(ux * 73856093ULL ^ uy * 19349663ULL ^ uz * 83492791ULL);
}

// the source sends one point per voxel, inside that voxel, so both sides get the same key
MapStore::Key MapStore::key_of(const Point& p) const {
    return {static_cast<std::int32_t>(std::floor(p.x / voxel_size_)),
            static_cast<std::int32_t>(std::floor(p.y / voxel_size_)),
            static_cast<std::int32_t>(std::floor(p.z / voxel_size_))};
}

void MapStore::upsert(const Point& p) {
    const auto [it, added] = index_.try_emplace(key_of(p), points_.size());
    if (added) {
        points_.push_back(p);
    } else {
        points_[it->second] = p;
    }
}

void MapStore::reset(std::string frame_id, double voxel_size) {
    const std::lock_guard<std::mutex> lock(mutex_);
    frame_id_ = std::move(frame_id);
    voxel_size_ = voxel_size;
    run_id_.reset();
    seq_ = 0;
    has_hits_ = has_intensity_ = false;
    points_.clear();
    index_.clear();
    trajectory_.clear();
    dirty_ = false;
}

void MapStore::put_map(std::uint32_t run_id, std::uint32_t seq, const std::vector<Point>& points,
                       bool has_hits, bool has_intensity) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (run_id_ != run_id) {
        points_.clear();
        index_.clear();
        trajectory_.clear();
        run_id_ = run_id;
    }
    points_.reserve(points.size());
    index_.reserve(points.size());
    for (const Point& p : points) upsert(p);
    seq_ = seq;
    has_hits_ = has_hits;
    has_intensity_ = has_intensity;
    dirty_ = true;
}

bool MapStore::put_delta(std::uint32_t run_id, std::uint32_t seq, const std::vector<Point>& points) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (run_id_ != run_id || seq != seq_ + 1) return false;
    for (const Point& p : points) upsert(p);
    seq_ = seq;
    dirty_ = true;
    return true;
}

void MapStore::put_hits(std::uint32_t run_id, const std::vector<Point>& points) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (run_id_ != run_id) return;
    for (const Point& p : points) {
        const auto it = index_.find(key_of(p));
        if (it == index_.end()) continue;
        points_[it->second].hits = p.hits;
        dirty_ = true;
    }
}

void MapStore::put_poses(std::uint32_t run_id, const std::vector<Pose>& poses) {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (run_id_ != run_id) return;
    for (const Pose& p : poses) {
        if (append_pose(trajectory_, p)) dirty_ = true;
    }
}

std::optional<MapCopy> MapStore::take() {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!dirty_ || !run_id_) return std::nullopt;
    dirty_ = false;
    MapCopy copy;
    copy.snapshot.run_id = *run_id_;
    copy.snapshot.seq = seq_;
    copy.snapshot.frame_id = frame_id_;
    copy.snapshot.points = points_;
    copy.snapshot.has_hits = has_hits_;
    copy.snapshot.has_intensity = has_intensity_;
    copy.trajectory = trajectory_;
    return copy;
}

void MapStore::touch() {
    const std::lock_guard<std::mutex> lock(mutex_);
    dirty_ = true;
}

StoreStatus MapStore::status() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return {run_id_, seq_, points_.size(), trajectory_.size()};
}

}  // namespace vcortex
