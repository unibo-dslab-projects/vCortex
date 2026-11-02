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
    // offline_source: name of the saved map; set, /ingest refuses every source
    // an empty ingest_token skips the check
    WebServer(MapStore& store, const Config& config, std::optional<std::string> offline_source,
              std::string ingest_token);

    // blocks until SIGINT or SIGTERM
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

    void publish(const std::shared_ptr<const ModelOutput>& output);

    /// handles a new viewer connection: sends the full map on open, then delta updates
    void raw_open(Connection& conn);
    void raw_close(Connection& conn);
    /// broadcasts a full map to all viewers
    void raw_send_map(const std::string& map_frame);
    /// broadcasts a delta update to all viewers
    void raw_send_delta(const std::string& delta_frame);

    // digital twin management methods
    void twin_open(Connection& conn);
    void twin_close(Connection& conn);

    // robot ingest management methods
    void ingest_open(Connection& conn);
    void ingest_close(Connection& conn);
    void ingest_message(Connection& conn, const std::string& data, bool is_binary);
    void ingest_hello(Connection& conn, const std::string& text);
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
    Connection* ingest_ = nullptr;
    bool ingest_ready_ = false;  // hello accepted
    std::string source_;

    // last member: its thread calls publish, which uses everything above
    ModelWorker worker_;
};

}  // namespace vcortex
