#include "vcortex/protocol/raw_frame.hpp"

#include <cstring>

#include "vcortex/protocol/ingest.hpp"

namespace vcortex {

namespace {

std::string with_header(FrameKind kind, std::uint32_t run_id, std::uint32_t seq,
                        std::size_t count, std::size_t record_size) {
    std::string out(kFrameHeaderSize + count * record_size, '\0');
    const auto count32 = static_cast<std::uint32_t>(count);
    out[0] = static_cast<char>(kind);
    std::memcpy(&out[4], &run_id, sizeof run_id);
    std::memcpy(&out[8], &seq, sizeof seq);
    std::memcpy(&out[12], &count32, sizeof count32);
    return out;
}

void put_position(std::string& out, std::size_t at, const Point& p) {
    std::memcpy(&out[at], &p.x, sizeof p.x);
    std::memcpy(&out[at + 4], &p.y, sizeof p.y);
    std::memcpy(&out[at + 8], &p.z, sizeof p.z);
}

}  // namespace

std::string encode_raw_map(std::uint32_t run_id, std::uint32_t seq,
                           const std::vector<Point>& points) {
    std::string out = with_header(FrameKind::Map, run_id, seq, points.size(), kRawMapRecordSize);
    std::size_t at = kFrameHeaderSize;
    for (const Point& p : points) {
        put_position(out, at, p);
        at += kRawMapRecordSize;
    }
    return out;
}

std::string encode_raw_delta(std::uint32_t run_id, std::uint32_t seq,
                             const std::vector<Point>& points,
                             const std::vector<std::uint32_t>& indices) {
    std::string out =
        with_header(FrameKind::Delta, run_id, seq, points.size(), kRawDeltaRecordSize);
    std::size_t at = kFrameHeaderSize;
    for (std::size_t i = 0; i < points.size(); ++i) {
        put_position(out, at, points[i]);
        std::memcpy(&out[at + 12], &indices[i], sizeof indices[i]);
        at += kRawDeltaRecordSize;
    }
    return out;
}

}  // namespace vcortex
