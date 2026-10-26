#pragma once

#include <cstddef>

namespace vcortex {

// Step 15: /ws/raw frames (backend -> browser). Same 16-byte header as ingest (kinds Map and
// Delta), but each record is only x y z float32, so the browser can read it with
// new Float32Array(buffer, 16, count * 3).

inline constexpr std::size_t kRawRecordSize = 12;

}  // namespace vcortex
