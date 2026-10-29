#include "vcortex/io/config.hpp"

#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace vcortex {

namespace {

using Json = nlohmann::json;

// one json object: reads the keys asked for, then done() refuses every key nobody asked for
class Fields {
public:
    Fields(const Json& j, std::string prefix) : j_(j), prefix_(std::move(prefix)) {
        if (!j_.is_object()) {
            throw std::invalid_argument((prefix_.empty() ? "settings" : prefix_) +
                                        " must be an object");
        }
    }

    void number(const char* key, double& out) {
        if (const Json* v = find(key)) {
            if (!v->is_number()) throw bad(key, "a number");
            out = v->get<double>();
        }
    }

    template <class Int>
    void integer(const char* key, Int& out) {
        if (const Json* v = find(key)) {
            if (!v->is_number_integer()) throw bad(key, "an integer");
            const auto n = v->get<std::int64_t>();
            if (n < std::numeric_limits<Int>::min() || n > std::numeric_limits<Int>::max()) {
                throw bad(key, "an integer in range");
            }
            out = static_cast<Int>(n);
        }
    }

    template <class Read>
    void object(const char* key, Read read) {
        if (const Json* v = find(key)) {
            Fields inner(*v, path(key));
            read(inner);
            inner.done();
        }
    }

    bool has(const char* key) const { return j_.contains(key); }

    void done() const {
        for (const auto& item : j_.items()) {
            if (used_.count(item.key()) == 0) {
                throw std::invalid_argument(path(item.key()) + " is not a setting");
            }
        }
    }

private:
    const Json* find(const char* key) {
        used_.insert(key);
        const auto it = j_.find(key);
        return it == j_.end() ? nullptr : &*it;
    }

    std::string path(const std::string& key) const {
        return prefix_.empty() ? key : prefix_ + "." + key;
    }

    std::invalid_argument bad(const char* key, const char* want) const {
        return std::invalid_argument(path(key) + " must be " + want);
    }

    const Json& j_;
    std::string prefix_;
    std::set<std::string> used_;
};

void read_settings(Fields& f, ModelSettings& s) {
    f.number("wall_height", s.wall_height);
    f.integer("min_scans", s.min_scans);
    f.object("outliers", [&](Fields& o) {
        o.integer("mean_k", s.outlier_mean_k);
        o.number("stddev_mul", s.outlier_stddev_mul);
    });
    f.object("candidates", [&](Fields& o) {
        CandidateParams& c = s.candidates;
        o.number("band_lo", c.band_lo);
        o.number("band_top_margin", c.band_top_margin);
        o.number("cell", c.cell);
        o.integer("min_column", c.min_column);
        o.number("max_gap", c.max_gap);
        o.number("min_length", c.min_length);
        o.number("merge_distance", c.merge_distance);
    });
    f.object("walls", [&](Fields& o) {
        WallParams& w = s.walls;
        o.number("span_margin", w.span_margin);
        o.number("both_sides_range", w.both_sides_range);
        o.number("max_depth", w.max_depth);
        o.number("overlap_widen", w.overlap_widen);
        o.number("collinear_tolerance", w.collinear_tolerance);
        o.number("min_gap", w.min_gap);
        o.number("door_max", w.door_max);
        o.number("corner_gap", w.corner_gap);
        o.number("thickness", w.thickness);
        o.number("object_clearance", w.object_clearance);
        o.number("cluster_tolerance", w.cluster_tolerance);
        o.integer("cluster_min_points", w.cluster_min_points);
        o.number("against_wall_distance", w.against_wall_distance);
    });
    f.object("rooms", [&](Fields& o) {
        o.number("min_area", s.rooms.min_area);
        o.number("wall_touch_distance", s.rooms.wall_touch_distance);
    });
}

Json parse(const std::string& text) {
    try {
        return Json::parse(text);
    } catch (const Json::parse_error& e) {
        throw std::invalid_argument(std::string("not valid json: ") + e.what());
    }
}

void check(const ModelSettings& s) {
    if (const auto problem = validate(s)) throw std::invalid_argument(*problem);
}

}  // namespace

Config load_config(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("load_config: cannot read " + file.string());
    std::ostringstream text;
    text << in.rdbuf();
    try {
        const Json j = parse(text.str());
        Config cfg;
        Fields top(j, "");
        top.number("model_period_s", cfg.model_period_s);
        read_settings(top, cfg.model);
        top.done();
        check(cfg.model);
        if (!(cfg.model_period_s > 0)) throw std::invalid_argument("model_period_s must be > 0");
        return cfg;
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument("load_config: " + file.string() + ": " + e.what());
    }
}

ModelSettings apply_settings(const ModelSettings& current, const std::string& body) {
    const Json j = parse(body);
    ModelSettings next = current;
    Fields top(j, "");
    read_settings(top, next);
    top.done();
    check(next);
    // the file holds the deployment's start value; a height sent here is the user's own
    if (top.has("wall_height")) next.wall_height_source = HeightSource::User;
    return next;
}

}  // namespace vcortex
