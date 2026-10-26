#pragma once

#include <string>

#include "vcortex/model/model.hpp"

namespace vcortex {

// Step 12: the result in the shape of schema/twin.schema.json.
// Metres rounded to 3 decimals, null for unknown optional values.
std::string to_twin_json(const ModelResult& result);

}  // namespace vcortex
