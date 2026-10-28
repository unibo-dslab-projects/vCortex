#pragma once

#include <cstddef>

namespace vcortex {

// /ws/raw (backend -> browser): ingest header, records only x y z float32, so three.js takes
// new Float32Array(buffer, 16, count * 3) without a copy

inline constexpr std::size_t kRawRecordSize = 12;

}  // namespace vcortex
