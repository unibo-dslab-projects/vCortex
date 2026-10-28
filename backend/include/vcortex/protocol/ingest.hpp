#pragma once

#include <cstddef>
#include <cstdint>

namespace vcortex {

// Ingest protocol (source -> backend). Binary frame = 16-byte header, then `count` records whose
// size depends on the frame kind.
// Little-endian, because the robot (ARM64) and the laptop (x86-64) both are: neither side swaps
// bytes.

inline constexpr int kIngestProtocol = 1;
inline constexpr std::size_t kFrameHeaderSize = 16;

enum class FrameKind : std::uint8_t {
    Map = 1,    // every voxel; seq = the last delta included
    Delta = 2,  // new voxels; seq = this delta's number
    Hits = 3,   // voxels sent before whose hits changed; seq = the delta it belongs to
    Poses = 4,  // robot poses; seq unused (0)
};

// Byte 0 kind, bytes 1..3 zero, then run_id, seq, count as uint32. The zero bytes keep every
// uint32 4-byte aligned.
struct FrameHeader {
    FrameKind kind;
    std::uint32_t run_id;
    std::uint32_t seq;
    std::uint32_t count;
};

// Map, Delta and Hits record (same layout, so one decoder serves all three; Hits repeats the
// unchanged fields rather than needing a layout of its own):
//   0  x          float32   metres, source map frame
//   4  y          float32
//   8  z          float32
//  12  intensity  float32   as the lidar reports it; float, because the source values have
//                           fractions that 8 or 16 bits would round away
//  16  hits       uint16
//  18  (zero)     2 bytes   keeps the next record's floats 4-byte aligned
inline constexpr std::size_t kVoxelRecordSize = 20;

// Poses record:
//   0  time_ns    int64     source clock, nanoseconds since the Unix epoch; integer, because a
//                           double rounds today's epoch values to about 0.2 microseconds
//   8  x          float32   metres, source map frame
//  12  y          float32
//  16  z          float32
//  20  yaw        float32   radians about z, counter-clockwise from x (ROS REP 103)
// 24 bytes, so time_ns stays 8-byte aligned in every record after the 16-byte header.
inline constexpr std::size_t kPoseRecordSize = 24;

// 0 for an unknown kind: the decoder refuses the frame instead of guessing a size.
constexpr std::size_t record_size(FrameKind kind) {
    switch (kind) {
        case FrameKind::Map:
        case FrameKind::Delta:
        case FrameKind::Hits:
            return kVoxelRecordSize;
        case FrameKind::Poses:
            return kPoseRecordSize;
    }
    return 0;
}

}  // namespace vcortex
