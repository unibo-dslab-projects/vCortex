#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// /ws/raw (backend -> browser): the ingest header, then records

// map: x y z float32, in the order of the backend's map, so three.js takes
// new Float32Array(buffer, 16, count * 3) without a copy and the n-th record has index n
inline constexpr std::size_t kRawMapRecordSize = 12;

// delta: x y z float32, then the voxel's index uint32: the viewer replaces the point there,
// or extends its list when the index is past the end; a voxel the source updates must not
// be appended twice
inline constexpr std::size_t kRawDeltaRecordSize = 16;

std::string encode_raw_map(std::uint32_t run_id, std::uint32_t seq,
                           const std::vector<Point>& points);

// indices: one per point, as MapStore::put_delta returns them
std::string encode_raw_delta(std::uint32_t run_id, std::uint32_t seq,
                             const std::vector<Point>& points,
                             const std::vector<std::uint32_t>& indices);

}  // namespace vcortex
