#include "vcortex/connectors/ply.hpp"

#include <cstdint>
#include <cstring>
#include <string>

namespace vcortex {

namespace {

// byte by byte, not a memcpy of the whole value: the file stays little-endian on any host
void put_le(std::string& out, std::uint32_t v, int bytes) {
    for (int i = 0; i < bytes; ++i) out.push_back(static_cast<char>((v >> (8 * i)) & 0xFFU));
}

void put_float(std::string& out, float f) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof bits);
    put_le(out, bits, 4);
}

}  // namespace

// x y z hits, not the whole Point: intensity is the first reading per voxel only, too noisy to
// show, and leaving it out keeps 14 bytes per point instead of 18
std::string to_ply(const ModelResult& result) {
    const std::vector<Point>& pts = result.cleaned;
    std::string out =
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment frame_id " + result.building.frame_id + "\n"
        "element vertex " + std::to_string(pts.size()) + "\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property ushort hits\n"
        "end_header\n";
    out.reserve(out.size() + pts.size() * 14);
    for (const Point& p : pts) {
        put_float(out, p.x);
        put_float(out, p.y);
        put_float(out, p.z);
        put_le(out, p.hits, 2);
    }
    return out;
}

}  // namespace vcortex
