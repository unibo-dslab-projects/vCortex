#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

#include <crow.h>

// TODO: --config and --map, full route table, map store and model worker
int main(int argc, char** argv) {
    std::uint16_t port = 9002;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else {
            std::cerr << "usage: vcortex [--port 9002]\n";
            return EXIT_FAILURE;
        }
    }

    crow::SimpleApp app;
    CROW_ROUTE(app, "/api/status")([] { return crow::json::wvalue{{"mode", "idle"}}; });

    // crow stops cleanly on SIGINT and SIGTERM, so `docker stop` needs nothing extra
    app.port(port).multithreaded().run();
    return EXIT_SUCCESS;
}
