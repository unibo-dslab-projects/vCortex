#pragma once

#include <string>

#include "vcortex/model/model.hpp"

namespace vcortex {

/// Converts a ModelResult to a JSON string
std::string to_json(const ModelResult& result);

}  // namespace vcortex
