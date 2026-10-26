#pragma once

#include <cstddef>
#include <cstdint>

namespace vcortex {

// Step 14: the ingest protocol (source -> backend). All little-endian.
// Binary frame = 16-byte header, then `count` records. The README is the reference.

inline constexpr int kIngestProtocol = 1;
inline constexpr std::size_t kFrameHeaderSize = 16;
inline constexpr std::size_t kIngestRecordSize = 16;

enum class FrameKind : std::uint8_t {
    Map = 1,    // every voxel; seq = the last delta included
    Delta = 2,  // new voxels; seq = this delta's number
    Hits = 3,   // voxels sent before whose hits changed; seq = the delta it belongs to
    Poses = 4,  // robot positions; seq unused (0)
};

// Byte 0 kind, bytes 1..3 zero, then run_id, seq, count as uint32.
struct FrameHeader {
    FrameKind kind;
    std::uint32_t run_id;
    std::uint32_t seq;
    std::uint32_t count;
};

// Map, Delta and Hits records: x y z float32, hits uint16, 2 zero bytes.
// Poses records: x y z yaw float32.

}  // namespace vcortex
