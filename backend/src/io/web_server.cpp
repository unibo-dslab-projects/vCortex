#include "vcortex/io/web_server.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include <crow.h>
#include <nlohmann/json.hpp>

#include "vcortex/connectors/connector.hpp"
#include "vcortex/protocol/ingest.hpp"
#include "vcortex/protocol/raw_frame.hpp"

namespace vcortex {

namespace {

using Json = nlohmann::ordered_json;

// over this many delta bytes in one second a viewer counts as slow: crow exposes no send queue
constexpr std::size_t kRawBudget = 8 * 1024 * 1024;
// a map of 10 million voxels; without a limit one bad frame could fill the memory
constexpr std::uint64_t kMaxIngestFrame = 256 * 1024 * 1024;
// rfc 6455 leaves 1013 to the iana registry, so crow has no name for it
constexpr std::uint16_t kCloseTryLater = 1013;

crow::response json_response(int code, const Json& body) {
    crow::response res(code, body.dump());
    res.set_header("Content-Type", "application/json");
    return res;
}

crow::response error_response(int code, const std::string& message) {
    return json_response(code, {{"error", message}});
}

Json settings_json(const ModelSettings& s) {
    return {{"wall_height", s.wall_height},
            {"wall_height_source",
             s.wall_height_source == HeightSource::User ? "user" : "default"}};
}

Json report_json(const ModelReport& r) {
    return {{"points_in", r.points_in},
            {"after_outliers", r.after_outliers},
            {"tilt_deg", r.tilt_deg},
            {"room_space_area_m2", r.room_space_area},
            {"room_space_leaked", r.room_space_leaked},
            {"yaw_deg", r.yaw_deg},
            {"poses", r.poses},
            {"candidates", r.candidates},
            {"walls", r.walls},
            {"rooms", r.rooms},
            {"objects", r.objects},
            {"duration_ms", r.duration_ms}};
}

const Connector* find_connector(const std::string& id) {
    for (const Connector& c : connectors()) {
        if (c.id == id) return &c;
    }
    return nullptr;
}

void send_output(crow::websocket::connection& conn, const Connector& connector,
                 const std::string& bytes) {
    if (connector.binary) {
        conn.send_binary(bytes);
    } else {
        conn.send_text(bytes);
    }
}

std::string full_map_frame(const MapStore& store) {
    const std::optional<MapCopy> copy = store.peek();
    if (!copy) return {};
    return encode_raw(FrameKind::Map, copy->snapshot.run_id, copy->snapshot.seq,
                      copy->snapshot.points);
}

// catch-ups average at most kRawBudget per second, however large the map
std::chrono::steady_clock::time_point next_full_time(std::chrono::steady_clock::time_point now,
                                                     std::size_t map_frame_bytes) {
    const std::chrono::duration<double> transfer_time(static_cast<double>(map_frame_bytes) /
                                                      static_cast<double>(kRawBudget));
    return now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(transfer_time);
}

}  // namespace

WebServer::WebServer(MapStore& store, const Config& config,
                     std::optional<std::string> offline_source, std::string ingest_token)
    : store_(store),
      offline_source_(std::move(offline_source)),
      ingest_token_(std::move(ingest_token)),
      source_(offline_source_.value_or("")),
      worker_(store, config.model, config.model_period_s,
              [this](const std::shared_ptr<const ModelOutput>& output) { publish(output); }) {}

void WebServer::run(std::uint16_t port) {
    crow::SimpleApp app;

    CROW_ROUTE(app, "/api/formats")([] {
        Json out = Json::array();
        for (const Connector& c : connectors()) {
            out.push_back({{"id", c.id}, {"label", c.label}, {"binary", c.binary}});
        }
        return json_response(200, out);
    });

    CROW_ROUTE(app, "/api/settings")
        .methods(crow::HTTPMethod::Get)([this] {
            return json_response(200, settings_json(worker_.settings()));
        });

    CROW_ROUTE(app, "/api/settings")
        .methods(crow::HTTPMethod::Put)([this](const crow::request& req) {
            const std::lock_guard<std::mutex> lock(settings_mutex_);
            try {
                const ModelSettings next = apply_settings(worker_.settings(), req.body);
                worker_.set_settings(next);
                return json_response(200, settings_json(next));
            } catch (const std::invalid_argument& e) {
                return error_response(400, e.what());
            }
        });

    CROW_ROUTE(app, "/api/status")([this] {
        const StoreStatus st = store_.status();
        const std::shared_ptr<const ModelOutput> latest = worker_.latest();
        Json connected, source, raw, twin;
        {
            const std::lock_guard<std::mutex> lock(clients_mutex_);
            connected = ingest_ != nullptr;
            source = source_;
            raw = raw_.size();
            std::size_t subscribers = 0;
            for (const auto& [id, conns] : twin_) subscribers += conns.size();
            twin = subscribers;
        }
        return json_response(
            200, {{"mode", offline_source_ ? "offline" : "live"},
                  {"source", source},
                  {"connected", connected},
                  {"run_id", st.run_id ? Json(*st.run_id) : Json(nullptr)},
                  {"seq", st.seq},
                  {"points", st.points},
                  {"model", latest ? report_json(latest->report) : Json(nullptr)},
                  {"clients", {{"raw", raw}, {"twin", twin}}}});
    });

    CROW_ROUTE(app, "/api/twin/<string>")([this](const std::string& id) {
        const Connector* connector = find_connector(id);
        if (connector == nullptr) return error_response(404, "unknown format: " + id);
        const std::shared_ptr<const ModelOutput> latest = worker_.latest();
        // a connector can fail alone (model worker logs it), so a run may lack this format
        if (!latest || latest->by_format.count(id) == 0) {
            return error_response(503, "no model output yet");
        }
        crow::response res(200, latest->by_format.at(id));
        res.set_header("Content-Type", connector->mime);
        res.set_header("Content-Disposition", "attachment; filename=\"twin-" +
                                                  std::to_string(latest->seq) +
                                                  connector->extension + "\"");
        return res;
    });

    CROW_WEBSOCKET_ROUTE(app, "/ws/raw")
        .onopen([this](Connection& conn) { raw_open(conn); })
        .onclose([this](Connection& conn, const std::string&, std::uint16_t) { raw_close(conn); });

    CROW_WEBSOCKET_ROUTE(app, "/ws/twin")
        .onaccept([](const crow::request& req, std::optional<crow::response>& res, void** data) {
            const char* format = req.url_params.get("format");
            const Connector* connector = format ? find_connector(format) : nullptr;
            if (connector == nullptr) {
                res = crow::response(404);
                return;
            }
            *data = const_cast<Connector*>(connector);
        })
        .onopen([this](Connection& conn) { twin_open(conn); })
        .onclose([this](Connection& conn, const std::string&, std::uint16_t) { twin_close(conn); });

    CROW_WEBSOCKET_ROUTE(app, "/ingest")
        .max_payload(kMaxIngestFrame)
        .onopen([this](Connection& conn) { ingest_open(conn); })
        .onmessage([this](Connection& conn, const std::string& data, bool is_binary) {
            ingest_message(conn, data, is_binary);
        })
        .onclose(
            [this](Connection& conn, const std::string&, std::uint16_t) { ingest_close(conn); });

    worker_.start();
    // crow stops cleanly on SIGINT and SIGTERM, so `docker stop` needs nothing extra
    app.port(port).multithreaded().run();
    // before app goes: publish must not reach connections crow has destroyed
    worker_.stop();
}

void WebServer::publish(const std::shared_ptr<const ModelOutput>& output) {
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    // a new source cleared the worker after this run ended: the output shows the old map
    if (output != worker_.latest()) return;
    for (const Connector& c : connectors()) {
        const auto bytes = output->by_format.find(c.id);
        const auto subscribers = twin_.find(c.id);
        if (bytes == output->by_format.end() || subscribers == twin_.end()) continue;
        for (Connection* conn : subscribers->second) send_output(*conn, c, bytes->second);
    }
}

void WebServer::raw_open(Connection& conn) {
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    RawClient client;
    client.window_start = std::chrono::steady_clock::now();
    client.next_full_allowed = client.window_start;
    std::string map_frame = full_map_frame(store_);
    if (!map_frame.empty()) {
        client.next_full_allowed = next_full_time(client.window_start, map_frame.size());
        conn.send_binary(std::move(map_frame));
    }
    raw_.emplace(&conn, client);
}

void WebServer::raw_close(Connection& conn) {
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    raw_.erase(&conn);
}

// every viewer, slow or not: a map replaces what it holds, so it also catches up
void WebServer::raw_send_map(const std::string& map_frame) {
    const auto now = std::chrono::steady_clock::now();
    for (auto& [conn, client] : raw_) {
        client.need_full = false;
        client.next_full_allowed = next_full_time(now, map_frame.size());
        conn->send_binary(map_frame);
    }
}

// a slow viewer gets the full map with the next delta after its window ends
void WebServer::raw_send_delta(const std::string& delta_frame) {
    const auto now = std::chrono::steady_clock::now();
    std::string catch_up_frame;
    for (auto& [conn, client] : raw_) {
        if (now - client.window_start >= std::chrono::seconds(1)) {
            client.window_start = now;
            client.delta_bytes_sent = 0;
        }
        if (client.need_full) {
            if (client.delta_bytes_sent > 0 || now < client.next_full_allowed) continue;
            if (catch_up_frame.empty()) catch_up_frame = full_map_frame(store_);
            client.next_full_allowed = next_full_time(now, catch_up_frame.size());
            client.need_full = false;
            conn->send_binary(catch_up_frame);
        } else if (client.delta_bytes_sent + delta_frame.size() > kRawBudget) {
            client.need_full = true;
        } else {
            client.delta_bytes_sent += delta_frame.size();
            conn->send_binary(delta_frame);
        }
    }
}

void WebServer::twin_open(Connection& conn) {
    const auto* connector = static_cast<const Connector*>(conn.userdata());
    const std::shared_ptr<const ModelOutput> latest = worker_.latest();
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    twin_[connector->id].insert(&conn);
    if (!latest) return;
    const auto bytes = latest->by_format.find(connector->id);
    if (bytes != latest->by_format.end()) send_output(conn, *connector, bytes->second);
}

void WebServer::twin_close(Connection& conn) {
    const auto* connector = static_cast<const Connector*>(conn.userdata());
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    twin_[connector->id].erase(&conn);
}

void WebServer::ingest_open(Connection& conn) {
    if (offline_source_) conn.close("offline mode", kCloseTryLater);
}

void WebServer::ingest_close(Connection& conn) {
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    if (&conn == ingest_) ingest_ = nullptr;
}

// a connection becomes the source only with a valid hello
void WebServer::ingest_message(Connection& conn, const std::string& data, bool is_binary) {
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    if (&conn == ingest_) {
        if (is_binary) ingest_frame(conn, data);
    } else if (is_binary) {
        conn.close("hello first", crow::websocket::PolicyViolated);
    } else {
        ingest_hello(conn, data);
    }
}

void WebServer::ingest_hello(Connection& conn, const std::string& text) {
    try {
        const nlohmann::json hello = nlohmann::json::parse(text);
        if (hello.at("type") != "hello") throw std::invalid_argument("hello first");
        if (hello.at("protocol").get<int>() != kIngestProtocol) {
            throw std::invalid_argument("unsupported protocol");
        }
        if (!ingest_token_.empty() && hello.value("token", "") != ingest_token_) {
            throw std::invalid_argument("bad token");
        }
        const auto frame_id = hello.at("frame_id").get<std::string>();
        const auto voxel_size = hello.at("voxel_size").get<double>();
        if (frame_id.empty() || !(voxel_size > 0.0)) {
            throw std::invalid_argument("bad frame_id or voxel_size");
        }
        source_ = hello.value("source", "");
        store_.reset(frame_id, voxel_size);
    } catch (const std::exception& e) {
        conn.close(e.what(), crow::websocket::PolicyViolated);
        return;
    }
    // the newer source wins: a robot that lost wi-fi never closes its socket, and refusing
    // the next one would lock every source out until a restart
    if (ingest_ != nullptr) ingest_->close("replaced by a newer source", kCloseTryLater);
    ingest_ = &conn;
    worker_.clear();
    // an empty message means "no twin yet": viewers drop the old source's twin
    for (const auto& [format, viewers] : twin_) {
        for (Connection* viewer : viewers) viewer->send_text("");
    }
    conn.send_text(R"({"type":"welcome"})");
}

void WebServer::ingest_frame(Connection& conn, const std::string& bytes) {
    IngestFrame frame;
    try {
        frame = decode_frame(bytes);
    } catch (const std::invalid_argument& e) {
        conn.close(e.what(), crow::websocket::PolicyViolated);
        return;
    }
    const FrameHeader& h = frame.header;
    switch (h.kind) {
        case FrameKind::Map:
            store_.put_map(h.run_id, h.seq, frame.points);
            raw_send_map(encode_raw(FrameKind::Map, h.run_id, h.seq, frame.points));
            break;
        case FrameKind::Delta:
            if (store_.put_delta(h.run_id, h.seq, frame.points)) {
                raw_send_delta(encode_raw(FrameKind::Delta, h.run_id, h.seq, frame.points));
            } else {
                conn.send_text(R"({"type":"resync","reason":"seq gap"})");
            }
            break;
        case FrameKind::Poses:
            store_.put_poses(h.run_id, frame.poses);
            break;
    }
}

}  // namespace vcortex
