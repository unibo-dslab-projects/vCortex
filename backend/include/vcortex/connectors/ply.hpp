#pragma once

#include <string>

#include "vcortex/model/model.hpp"

namespace vcortex {

// Step 13: the cleaned cloud, binary little-endian, float x y z + ushort hits.
std::string to_ply(const ModelResult& result);

}  // namespace vcortex
