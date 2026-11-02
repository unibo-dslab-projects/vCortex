#include "vcortex/protocol/raw_frame.hpp"

#include <cstring>

namespace vcortex {

std::string encode_raw(FrameKind kind, std::uint32_t run_id, std::uint32_t seq,
                       const std::vector<Point>& points) {
    std::string out(kFrameHeaderSize + points.size() * kRawRecordSize, '\0');
    const auto count = static_cast<std::uint32_t>(points.size());
    out[0] = static_cast<char>(kind);
    std::memcpy(&out[4], &run_id, sizeof run_id);
    std::memcpy(&out[8], &seq, sizeof seq);
    std::memcpy(&out[12], &count, sizeof count);
    std::size_t at = kFrameHeaderSize;
    for (const Point& p : points) {
        std::memcpy(&out[at], &p.x, sizeof p.x);
        std::memcpy(&out[at + 4], &p.y, sizeof p.y);
        std::memcpy(&out[at + 8], &p.z, sizeof p.z);
        at += kRawRecordSize;
    }
    return out;
}

}  // namespace vcortex
