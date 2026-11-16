#pragma once

#include <filesystem>
#include <string>

#include "vcortex/domain/settings.hpp"

namespace vcortex {

struct Config {
    ModelSettings model;
    double model_period_s = 3.0;  // model worker wake-up; not a model setting, so not in the PUT
};

/// Load the configuration from the given JSON file, throwing on any error.
Config load_config(const std::filesystem::path& file);

/// Apply the settings from the given JSON body to the current settings, throwing on any error.
ModelSettings apply_settings(const ModelSettings& current, const std::string& body);

}  // namespace vcortex
