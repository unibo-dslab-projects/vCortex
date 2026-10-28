#include "vcortex/model/clean.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>

#include <pcl/filters/statistical_outlier_removal.h>

#include "pcl_bridge.hpp"

namespace vcortex {

std::vector<Point> drop_transient(const std::vector<Point>& in, std::uint16_t min_scans) {
    std::vector<Point> out;
    // Most voxels of a mapped room are structure, so the output is close to the input size.
    out.reserve(in.size());
    std::copy_if(in.begin(), in.end(), std::back_inserter(out),
                 [min_scans](const Point& p) { return p.hits >= min_scans; });
    return out;
}

std::vector<Point> drop_outliers(const std::vector<Point>& in, int mean_k, double stddev_mul) {
    // PCL searches mean_k + 1 neighbours (the first is the point itself), and its k-search fails
    // when the cloud has fewer points than that. Too few points carry no statistics anyway.
    if (in.size() <= static_cast<std::size_t>(mean_k)) {
        return in;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(to_pcl(in));
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul);
    pcl::Indices kept;  // ascending, so the input order is kept
    sor.filter(kept);
    return pick(in, kept);
}

}  // namespace vcortex
