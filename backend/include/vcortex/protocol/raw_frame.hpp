#pragma once

#include <cstddef>

namespace vcortex {

// /ws/raw frames (backend -> browser). Same 16-byte header as ingest (kinds Map and Delta), but
// hits are left out: each record is only x y z float32, so the browser hands the buffer to
// three.js without a copy, as new Float32Array(buffer, 16, count * 3).

inline constexpr std::size_t kRawRecordSize = 12;

}  // namespace vcortex
