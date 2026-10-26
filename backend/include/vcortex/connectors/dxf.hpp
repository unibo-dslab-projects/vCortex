#pragma once

#include <string>

#include "vcortex/model/model.hpp"

namespace vcortex {

// Step 13: R2000 (AC1015) floor plan in metres. Layers WALLS, ROOMS, OBJECTS.
// Objects are four LINE entities, never an LWPOLYLINE (CrowdVision would import it as a room).
std::string to_dxf(const ModelResult& result);

}  // namespace vcortex
