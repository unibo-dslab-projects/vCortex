#pragma once

#include <cstdint>
#include <string>

#include "vcortex/io/map_store.hpp"
#include "vcortex/io/model_worker.hpp"

namespace vcortex {

// TODO: /ingest, /ws/raw, /ws/twin?format=<id>, /api/formats and /api/settings
// one port for every route: robot, browser and docker need only 9002
class WebServer {
public:
    // mode and source only name what this process serves, for /api/status
    WebServer(MapStore& store, ModelWorker& worker, std::string mode, std::string source);

    // blocks until SIGINT or SIGTERM
    void run(std::uint16_t port);

private:
    MapStore& store_;
    ModelWorker& worker_;
    const std::string mode_, source_;
};

}  // namespace vcortex
