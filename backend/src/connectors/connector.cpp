#include "vcortex/connectors/connector.hpp"

#include "vcortex/connectors/dxf.hpp"
#include "vcortex/connectors/json.hpp"
#include "vcortex/connectors/ply.hpp"

namespace vcortex {

const std::vector<Connector>& connectors() {
    static const std::vector<Connector> all = {
        {"twin-json", "Digital twin (JSON)", "application/json", ".json", false, to_json},
        {"dxf", "Floor plan (DXF)", "image/vnd.dxf", ".dxf", false, to_dxf},
        // ply has no registered media type
        {"ply", "Cleaned cloud (PLY)", "application/octet-stream", ".ply", true, to_ply},
    };
    return all;
}

}  // namespace vcortex
