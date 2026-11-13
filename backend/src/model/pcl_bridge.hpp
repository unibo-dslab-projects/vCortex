#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/types.h>

#include "vcortex/domain/lidar_point.hpp"

namespace vcortex {

// one way: pcl returns indices and callers keep the original Points, so intensity survives
// without a custom pcl point type
inline pcl::PointCloud<pcl::PointXYZ>::Ptr to_pcl(const std::vector<LidarPoint>& in) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(in.size());
    for (const LidarPoint& p : in) {
        cloud->push_back(pcl::PointXYZ(p.position.x(), p.position.y(), p.position.z()));
    }
    return cloud;
}

inline std::vector<LidarPoint> pick(const std::vector<LidarPoint>& in, const pcl::Indices& indices) {
    std::vector<LidarPoint> out;
    out.reserve(indices.size());
    for (const pcl::index_t i : indices) {
        out.push_back(in[static_cast<std::size_t>(i)]);
    }
    return out;
}

}  // namespace vcortex
