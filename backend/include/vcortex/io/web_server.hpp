#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>

#include "vcortex/io/config.hpp"
#include "vcortex/io/map_store.hpp"
#include "vcortex/io/model_worker.hpp"

namespace crow::websocket {
struct connection;
}

namespace vcortex {

// one port for every route: robot, browser and docker need only 9002
class WebServer {
public:
    /// Creates the server. With offline_source set (the saved map's name), /ingest refuses every
    /// source; without it, a source with a valid hello replaces the one before. An empty
    /// ingest_token skips the token check.
    WebServer(MapStore& store, const Config& config, std::optional<std::string> offline_source,
              std::string ingest_token);

    /// Serves every route on the given port, blocking until SIGINT or SIGTERM.
    void run(std::uint16_t port);

private:
    using Connection = crow::websocket::connection;
    struct RawClient {
        std::chrono::steady_clock::time_point window_start;
        std::size_t delta_bytes_sent = 0;  // since window_start
        bool need_full = false;            // skipped a delta, so only a full map can catch it up
        // a full map larger than the budget must not read as a slow viewer, so it stays out of
        // delta_bytes_sent and only delays the next catch-up
        std::chrono::steady_clock::time_point next_full_allowed;
    };

    /// Sends a new model output to the twin viewers of each format, unless a new source made it stale.
    void publish(const std::shared_ptr<const ModelOutput>& output);

    /// Adds a new point cloud viewer and sends it the full map.
    void raw_open(Connection& conn);
    /// Removes a point cloud viewer.
    void raw_close(Connection& conn);
    /// Sends a full map to every point cloud viewer, which also catches up the slow ones.
    void raw_send_map(const std::string& map_frame);
    /// Sends a delta to every point cloud viewer; a viewer over its byte budget gets a full map later instead.
    void raw_send_delta(const std::string& delta_frame);

    /// Adds a twin viewer for the connection's format and sends it the last output.
    void twin_open(Connection& conn);
    /// Removes a twin viewer.
    void twin_close(Connection& conn);

    /// Refuses the robot connection in offline mode.
    void ingest_open(Connection& conn);
    /// Forgets the source if the closed connection was it.
    void ingest_close(Connection& conn);
    /// Routes a robot message: frames from the source, a hello from anyone else.
    void ingest_message(Connection& conn, const std::string& data, bool is_binary);
    /// Checks the hello and, if valid, makes the connection the new source, replacing the old one.
    void ingest_hello(Connection& conn, const std::string& text);
    /// Applies a map, delta or poses frame to the store and forwards it to the point cloud viewers.
    void ingest_frame(Connection& conn, const std::string& bytes);

    MapStore& store_;
    const std::optional<std::string> offline_source_;
    const std::string ingest_token_;

    std::mutex settings_mutex_;  // two PUTs read then write: without it one update is lost

    // also held while a frame is applied and sent, so a joining viewer sees each delta either
    // inside its full map or after it, never neither
    std::mutex clients_mutex_;
    std::map<Connection*, RawClient> raw_;
    std::map<std::string, std::set<Connection*>> twin_;  // connector id -> subscribers
    Connection* ingest_ = nullptr;  // the connection whose hello was accepted last
    std::string source_;

    // last member: its thread calls publish, which uses everything above
    ModelWorker worker_;
};

}  // namespace vcortex
