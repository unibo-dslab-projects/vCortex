#pragma once

#include <cstddef>
#include <cstdint>

namespace vcortex {

// Ingest protocol (source -> backend). Binary frame = 16-byte header, then `count` records.
// Little-endian, because the robot (ARM64) and the laptop (x86-64) both are: neither side swaps
// bytes.

inline constexpr int kIngestProtocol = 1;
inline constexpr std::size_t kFrameHeaderSize = 16;
inline constexpr std::size_t kIngestRecordSize = 16;

enum class FrameKind : std::uint8_t {
    Map = 1,    // every voxel; seq = the last delta included
    Delta = 2,  // new voxels; seq = this delta's number
    Hits = 3,   // voxels sent before whose hits changed; seq = the delta it belongs to
    Poses = 4,  // robot positions; seq unused (0)
};

// Byte 0 kind, bytes 1..3 zero, then run_id, seq, count as uint32. The zero bytes keep every
// uint32 4-byte aligned, so the receiver can read the fields straight from the buffer.
struct FrameHeader {
    FrameKind kind;
    std::uint32_t run_id;
    std::uint32_t seq;
    std::uint32_t count;
};

// Map, Delta and Hits records: x y z float32, hits uint16, 2 zero bytes.
// Poses records: x y z yaw float32.

}  // namespace vcortex
