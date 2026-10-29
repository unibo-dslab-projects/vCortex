#include "vcortex/model/clean.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>

#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl_bridge.hpp"

namespace vcortex {

std::vector<Point> drop_transient(const std::vector<Point>& in, std::uint16_t min_scans) {
    std::vector<Point> out;
    // most voxels are structure, so output is near input size
    out.reserve(in.size());
    std::copy_if(in.begin(), in.end(), std::back_inserter(out),
                 [min_scans](const Point& p) { return p.hits >= min_scans; });
    return out;
}

std::vector<Point> drop_outliers(const std::vector<Point>& in, int mean_k, double stddev_mul) {
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
