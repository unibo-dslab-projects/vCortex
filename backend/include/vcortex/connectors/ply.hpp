#pragma once

#include <string>

#include "vcortex/model/model.hpp"

namespace vcortex {

/// Converts a ModelResult to a PLY string
std::string to_ply(const ModelResult& result);

}  // namespace vcortex
