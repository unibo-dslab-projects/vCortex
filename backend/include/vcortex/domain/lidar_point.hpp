#pragma once

#include <cstdint>

#include <Eigen/Core>

namespace vcortex {

struct LidarPoint {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    float intensity = 0.0F;
    std::uint32_t num_scan_seen = 0;
};

}  // namespace vcortex
