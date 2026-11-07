#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/domain/snapshot.hpp"

namespace vcortex {

// source -> backend: 16-byte header, then `count` records sized by kind
// little-endian: robot (arm64) and laptop (x86-64) both are, so no byte swaps

inline constexpr int kIngestProtocol = 2;
inline constexpr std::size_t kFrameHeaderSize = 16;

enum class FrameKind : std::uint8_t {
    Map = 1,    // every voxel; seq = last delta included
    Delta = 2,  // new and updated voxels; replace by key; seq = this delta
    Poses = 3,  // robot poses; seq unused (0)
};

// byte 0 kind, bytes 1..3 zero (keep uint32s aligned), then run_id, seq, count as uint32
struct FrameHeader {
    FrameKind kind;
    std::uint32_t run_id;
    std::uint32_t seq;
    std::uint32_t count;
};

// map, delta: x y z intensity float32 (the voxel's mean; intensity has fractions), then
// num_scan_seen uint32
inline constexpr std::size_t kVoxelRecordSize = 20;

// poses: time_ns int64 (unix ns; a double loses ~0.2 us), then x y z yaw float32
// 24 bytes keep time_ns 8-byte aligned after the 16-byte header
inline constexpr std::size_t kPoseRecordSize = 24;

// 0 for unknown kind: the decoder refuses the frame, never guesses a size
constexpr std::size_t record_size(FrameKind kind) {
    switch (kind) {
        case FrameKind::Map:
        case FrameKind::Delta:
            return kVoxelRecordSize;
        case FrameKind::Poses:
            return kPoseRecordSize;
    }
    return 0;
}

// points for map, delta; poses for poses
struct IngestFrame {
    FrameHeader header;
    std::vector<Point> points;
    std::vector<Pose> poses;
};

// throws std::invalid_argument on a short frame, an unknown kind or a wrong size
IngestFrame decode_frame(const std::string& bytes);

}  // namespace vcortex
