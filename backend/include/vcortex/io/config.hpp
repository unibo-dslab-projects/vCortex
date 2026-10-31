#pragma once

#include <filesystem>
#include <string>

#include "vcortex/domain/settings.hpp"

namespace vcortex {

struct Config {
    ModelSettings model;
    double model_period_s = 3.0;  // model worker wake-up; not a model setting, so not in the PUT
};

// missing keys keep their start values; unknown keys and wrong types throw, so a typo such as
// "wall_heigth" fails at start instead of silently running with the default
Config load_config(const std::filesystem::path& file);

// PUT /api/settings body over the current settings, same key names as config/model.json;
// throws std::invalid_argument with the offending key, current settings stay untouched
ModelSettings apply_settings(const ModelSettings& current, const std::string& body);

}  // namespace vcortex
