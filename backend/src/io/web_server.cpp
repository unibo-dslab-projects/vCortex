#include "vcortex/io/web_server.hpp"

#include <memory>
#include <utility>

#include <crow.h>
#include <nlohmann/json.hpp>

#include "vcortex/connectors/connector.hpp"

namespace vcortex {

namespace {

using Json = nlohmann::ordered_json;

crow::response json_response(int code, const Json& body) {
    crow::response res(code, body.dump());
    res.set_header("Content-Type", "application/json");
    return res;
}

crow::response error_response(int code, const std::string& message) {
    return json_response(code, {{"error", message}});
}

Json report_json(const ModelReport& r) {
    return {{"points_in", r.points_in},
            {"after_transient", r.after_transient},
            {"after_outliers", r.after_outliers},
            {"transient_skipped", r.transient_skipped},
            {"tilt_deg", r.tilt_deg},
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

}  // namespace

WebServer::WebServer(MapStore& store, ModelWorker& worker, std::string mode, std::string source)
    : store_(store), worker_(worker), mode_(std::move(mode)), source_(std::move(source)) {}

void WebServer::run(std::uint16_t port) {
    crow::SimpleApp app;

    CROW_ROUTE(app, "/api/status")([this] {
        const StoreStatus st = store_.status();
        const std::shared_ptr<const ModelOutput> latest = worker_.latest();
        return json_response(
            200, {{"mode", mode_},
                  {"source", source_},
                  {"connected", false},
                  {"run_id", st.run_id ? Json(*st.run_id) : Json(nullptr)},
                  {"seq", st.seq},
                  {"points", st.points},
                  {"model", latest ? report_json(latest->report) : Json(nullptr)}});
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

    // crow stops cleanly on SIGINT and SIGTERM, so `docker stop` needs nothing extra
    app.port(port).multithreaded().run();
}

}  // namespace vcortex
