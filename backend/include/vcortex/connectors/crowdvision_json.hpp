#pragma once

#include <string>

#include "vcortex/model/model.hpp"

namespace vcortex {

// Step 13: CrowdVision's building.schema.json. Rooms only, y up: (x, y, z) -> (x, z, -y).
// The only place in vCortex that knows CrowdVision exists.
std::string to_crowdvision_json(const ModelResult& result);

}  // namespace vcortex
