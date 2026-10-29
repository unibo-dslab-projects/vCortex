#include "vcortex/connectors/json.hpp"

#include <cmath>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace vcortex {

namespace {

// ordered: keys come out in schema order, so a person can read the file
using Json = nlohmann::ordered_json;

// major on a renamed or removed field, minor on a new optional one
constexpr const char* kSchemaVersion = "0.1.0";

// millimetres: point-lio's noise is centimetres, more digits are noise
// + 0.0 turns -0.0 into 0.0
double mm(double v) { return std::round(v * 1000.0) / 1000.0 + 0.0; }

Json xy(const Vec2& v) { return Json::array({mm(v.x), mm(v.y)}); }

// the schema wants null for unknown, never ""
Json text(const std::optional<std::string>& s) { return s ? Json(*s) : Json(nullptr); }

template <class T>
std::set<std::string> unique_ids(const std::vector<T>& items, const char* what) {
    std::set<std::string> ids;
    for (const T& item : items) {
        if (item.id.empty() || !ids.insert(item.id).second) {
            throw std::invalid_argument(std::string("to_json: empty or repeated ") + what +
                                        " id '" + item.id + "'");
        }
    }
    return ids;
}

void must_exist(const std::set<std::string>& ids, const std::optional<std::string>& ref,
                const std::string& owner) {
    if (ref && ids.count(*ref) == 0) {
        throw std::invalid_argument("to_json: " + owner + " points at missing id '" + *ref + "'");
    }
}

// a reader follows references without checking them, so a broken one stops here
void check(const Level& level) {
    const auto wall_ids = unique_ids(level.walls, "wall");
    const auto room_ids = unique_ids(level.rooms, "room");
    unique_ids(level.objects, "object");
    for (const Room& r : level.rooms) {
        if (r.outline.size() < 3) {
            throw std::invalid_argument("to_json: " + r.id + " outline has fewer than 3 points");
        }
        for (const std::string& w : r.wall_ids) must_exist(wall_ids, w, r.id);
    }
    for (const Object& o : level.objects) {
        must_exist(room_ids, o.room_id, o.id);
        must_exist(wall_ids, o.against_wall_id, o.id);
    }
}

Json level_json(const Level& level) {
    Json walls = Json::array();
    for (const Wall& w : level.walls) {
        walls.push_back({{"id", w.id},
                         {"start", xy(w.start)},
                         {"end", xy(w.end)},
                         {"thickness", mm(w.thickness)},
                         {"points", w.points},
                         {"coverage", w.coverage}});
    }
    Json rooms = Json::array();
    for (const Room& r : level.rooms) {
        Json outline = Json::array();
        for (const Vec2& v : r.outline) outline.push_back(xy(v));
        rooms.push_back({{"id", r.id},
                         {"name", text(r.name)},
                         {"closed", r.closed},
                         {"outline", std::move(outline)},
                         {"wall_ids", r.wall_ids}});
    }
    Json objects = Json::array();
    for (const Object& o : level.objects) {
        objects.push_back({{"id", o.id},
                           {"kind", o.kind},
                           {"room_id", text(o.room_id)},
                           {"against_wall_id", text(o.against_wall_id)},
                           {"center", xy(o.center)},
                           {"size", xy(o.size)},
                           {"yaw", o.yaw},
                           {"bottom_z", mm(o.bottom_z)},
                           {"top_z_seen", mm(o.top_z_seen)},
                           {"points", o.points}});
    }
    return {{"id", level.id},
            {"floor_z", mm(level.floor_z)},
            {"wall_height", mm(level.wall_height)},
            {"wall_height_source",
             level.wall_height_source == HeightSource::User ? "user" : "default"},
            {"walls", std::move(walls)},
            {"rooms", std::move(rooms)},
            {"objects", std::move(objects)}};
}

}  // namespace

std::string to_json(const ModelResult& result) {
    const Building& b = result.building;
    const ModelReport& rep = result.report;
    if (b.frame_id.empty()) throw std::invalid_argument("to_json: building has no frame_id");
    if (b.levels.empty()) throw std::invalid_argument("to_json: building has no level");

    Json levels = Json::array();
    for (const Level& level : b.levels) {
        check(level);
        levels.push_back(level_json(level));
    }
    const Json out = {{"schema_version", kSchemaVersion},
                      {"run_id", b.run_id},
                      {"seq", b.seq},
                      {"frame", {{"id", b.frame_id}, {"up", "z"}, {"unit", "m"}}},
                      {"report",
                       {{"points_in", rep.points_in},
                        {"after_transient", rep.after_transient},
                        {"after_outliers", rep.after_outliers},
                        {"transient_skipped", rep.transient_skipped},
                        {"floor_tilt_deg", rep.tilt_deg},
                        {"yaw_deg", rep.yaw_deg},
                        {"duration_ms", rep.duration_ms}}},
                      {"levels", std::move(levels)}};
    return out.dump();
}

}  // namespace vcortex
