#include "vcortex/protocol/ingest.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace vcortex {

namespace {

// memcpy: bytes from the socket are not aligned
template <typename T>
T read_at(const std::string& bytes, std::size_t offset) {
    T v;
    std::memcpy(&v, bytes.data() + offset, sizeof v);
    return v;
}

}  // namespace

IngestFrame decode_frame(const std::string& bytes) {
    if (bytes.size() < kFrameHeaderSize) throw std::invalid_argument("frame shorter than a header");

    IngestFrame frame;
    frame.header.kind = static_cast<FrameKind>(read_at<std::uint8_t>(bytes, 0));
    frame.header.run_id = read_at<std::uint32_t>(bytes, 4);
    frame.header.seq = read_at<std::uint32_t>(bytes, 8);
    frame.header.count = read_at<std::uint32_t>(bytes, 12);

    const std::size_t record = record_size(frame.header.kind);
    if (record == 0) throw std::invalid_argument("unknown frame kind");
    // 64-bit on both supported targets, so count * record cannot wrap
    const std::size_t count = frame.header.count;
    if (bytes.size() != kFrameHeaderSize + count * record) {
        throw std::invalid_argument("frame size does not match its count");
    }

    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t at = kFrameHeaderSize + i * record;
        if (frame.header.kind == FrameKind::Poses) {
            Pose p;
            p.time_ns = read_at<std::int64_t>(bytes, at);
            p.x = static_cast<double>(read_at<float>(bytes, at + 8));
            p.y = static_cast<double>(read_at<float>(bytes, at + 12));
            p.z = static_cast<double>(read_at<float>(bytes, at + 16));
            p.yaw = static_cast<double>(read_at<float>(bytes, at + 20));
            frame.poses.push_back(p);
        } else {
            Point p;
            p.x = read_at<float>(bytes, at);
            p.y = read_at<float>(bytes, at + 4);
            p.z = read_at<float>(bytes, at + 8);
            p.intensity = read_at<float>(bytes, at + 12);
            p.num_scan_seen = read_at<std::uint32_t>(bytes, at + 16);
            // a voxel key from nan is undefined behaviour
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                throw std::invalid_argument("frame holds a point that is not finite");
            }
            frame.points.push_back(p);
        }
    }
    return frame;
}

}  // namespace vcortex
