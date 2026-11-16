#include "vcortex/model/clean.hpp"

#include <cstddef>
#include <memory>
#include <vector>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/types.h>

namespace vcortex {

namespace {

// one way: pcl returns indices and callers keep the original points, so intensity survives
// without a custom pcl point type
pcl::PointCloud<pcl::PointXYZ>::Ptr to_pcl(const std::vector<LidarPoint>& points) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(points.size());
    for (const LidarPoint& point : points) {
        cloud->push_back(pcl::PointXYZ(point.position.x(), point.position.y(), point.position.z()));
    }
    return cloud;
}

std::vector<LidarPoint> select_points(const std::vector<LidarPoint>& points,
                                      const pcl::Indices& indices) {
    std::vector<LidarPoint> selected_points;
    selected_points.reserve(indices.size());
    for (const pcl::index_t index : indices) {
        selected_points.push_back(points[static_cast<std::size_t>(index)]);
    }
    return selected_points;
}

}  // namespace

std::vector<LidarPoint> drop_outliers(const std::vector<LidarPoint>& points, int mean_k,
                                      double stddev_mul) {
    // pcl searches mean_k + 1 neighbours (incl. the point) and fails with fewer points;
    // too few points carry no statistics anyway
    if (points.size() <= static_cast<std::size_t>(mean_k)) {
        return points;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(to_pcl(points));
    outlier_filter.setMeanK(mean_k);
    outlier_filter.setStddevMulThresh(stddev_mul);
    pcl::Indices kept_indices;  // ascending, keeps input order
    outlier_filter.filter(kept_indices);
    return select_points(points, kept_indices);
}

}  // namespace vcortex
