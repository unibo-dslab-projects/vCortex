#include <doctest/doctest.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include "vcortex/protocol/ingest.hpp"

namespace vcortex {
namespace {

// one pose at (x, 2, 0) facing yaw, laid out as the robot sends it
std::string one_pose_frame(float x, float yaw) {
    std::string bytes(kFrameHeaderSize + kPoseRecordSize, '\0');
    bytes[0] = static_cast<char>(FrameKind::Poses);
    const std::uint32_t count = 1;
    std::memcpy(&bytes[12], &count, sizeof count);
    const float position_and_yaw[4] = {x, 2.0F, 0.0F, yaw};
    std::memcpy(&bytes[kFrameHeaderSize + 8], position_and_yaw, sizeof position_and_yaw);
    return bytes;
}

TEST_CASE("a pose frame gives its pose") {
    const IngestFrame frame = decode_frame(one_pose_frame(1.5F, 0.25F));

    REQUIRE(frame.poses.size() == 1);
    CHECK(frame.poses[0].position.x() == 1.5);
    CHECK(frame.poses[0].yaw == 0.25);
}

TEST_CASE("a pose that is not finite is refused") {
    const float not_a_number = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    CHECK_THROWS_AS(decode_frame(one_pose_frame(not_a_number, 0.0F)), std::invalid_argument);
    CHECK_THROWS_AS(decode_frame(one_pose_frame(1.5F, infinity)), std::invalid_argument);
}

}  // namespace
}  // namespace vcortex
