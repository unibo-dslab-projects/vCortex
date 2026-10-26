#pragma once

#include <functional>
#include <string>
#include <vector>

#include "vcortex/model/model.hpp"

namespace vcortex {

// Step 13: one row per output format. Adding a format = one function + one row.
struct Connector {
    std::string id;         // "twin-json"; used in /ws/twin?format=<id> and /api/twin/<id>
    std::string label;      // "Digital twin (JSON)"; shown in the dropdown
    std::string mime;       // "application/json"
    std::string extension;  // ".json"; for the download name
    bool binary;            // true: send as a binary WebSocket frame
    std::function<std::string(const ModelResult&)> write;  // std::string also holds binary bytes
};

// The one place that lists every format.
const std::vector<Connector>& connectors();

}  // namespace vcortex
