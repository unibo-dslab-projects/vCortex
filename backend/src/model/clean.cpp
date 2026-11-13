#include "vcortex/model/clean.hpp"

#include <cstddef>

#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl_bridge.hpp"

namespace vcortex {

std::vector<LidarPoint> drop_outliers(const std::vector<LidarPoint>& in, int mean_k, double stddev_mul) {
    // pcl searches mean_k + 1 neighbours (incl. the point) and fails with fewer points;
    // too few points carry no statistics anyway
    if (in.size() <= static_cast<std::size_t>(mean_k)) {
        return in;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(to_pcl(in));
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul);
    pcl::Indices kept;  // ascending, keeps input order
    sor.filter(kept);
    return pick(in, kept);
}

}  // namespace vcortex
