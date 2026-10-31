#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include "vcortex/io/config.hpp"
#include "vcortex/io/map_store.hpp"
#include "vcortex/io/model_worker.hpp"
#include "vcortex/io/pcd_input.hpp"
#include "vcortex/io/web_server.hpp"

namespace {

using namespace vcortex;

// a saved pcd need not hold one point per 5 cm voxel: 0.1 mm keeps its points apart
constexpr double kOfflineVoxel = 0.0001;

void load_map(MapStore& store, const std::filesystem::path& file) {
    const OfflineInput in = load_offline(file);
    store.reset(in.snapshot.frame_id, kOfflineVoxel);
    store.put_map(in.snapshot.run_id, in.snapshot.seq, in.snapshot.points, in.snapshot.has_hits,
                  in.snapshot.has_intensity);
    store.put_poses(in.snapshot.run_id, in.trajectory);
}

int serve(std::uint16_t port, const std::optional<std::filesystem::path>& config_file,
          const std::optional<std::filesystem::path>& map_file) {
    const Config config = config_file ? load_config(*config_file) : Config{};

    MapStore store;
    if (map_file) load_map(store, *map_file);

    ModelWorker worker(store, config.model, config.model_period_s, nullptr);
    WebServer server(store, worker, map_file ? "offline" : "idle",
                     map_file ? map_file->filename().string() : "");
    worker.start();
    server.run(port);
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    std::uint16_t port = 9002;
    std::optional<std::filesystem::path> config_file, map_file;
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string_view arg = argv[i];
            if (arg == "--port" && i + 1 < argc) {
                port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
            } else if (arg == "--config" && i + 1 < argc) {
                config_file = argv[++i];
            } else if (arg == "--map" && i + 1 < argc) {
                map_file = argv[++i];
            } else {
                std::cerr << "usage: vcortex [--port 9002] [--config model.json] [--map map.pcd]\n";
                return EXIT_FAILURE;
            }
        }
        return serve(port, config_file, map_file);
    } catch (const std::exception& e) {
        std::cerr << "vcortex: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
