#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/types.h>

#include "vcortex/domain/point.hpp"

namespace vcortex {

// one way: pcl returns indices and callers keep the original Points, so hits and intensity
// survive without a custom pcl point type
inline pcl::PointCloud<pcl::PointXYZ>::Ptr to_pcl(const std::vector<Point>& in) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(in.size());
    for (const Point& p : in) {
        cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    return cloud;
}

inline std::vector<Point> pick(const std::vector<Point>& in, const pcl::Indices& indices) {
    std::vector<Point> out;
    out.reserve(indices.size());
    for (const pcl::index_t i : indices) {
        out.push_back(in[static_cast<std::size_t>(i)]);
    }
    return out;
}

}  // namespace vcortex
