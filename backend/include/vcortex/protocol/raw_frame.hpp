#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "vcortex/domain/point.hpp"
#include "vcortex/protocol/ingest.hpp"

namespace vcortex {

// /ws/raw (backend -> browser): ingest header, records only x y z float32, so three.js takes
// new Float32Array(buffer, 16, count * 3) without a copy

inline constexpr std::size_t kRawRecordSize = 12;

// kind is Map or Delta
std::string encode_raw(FrameKind kind, std::uint32_t run_id, std::uint32_t seq,
                       const std::vector<Point>& points);

}  // namespace vcortex
