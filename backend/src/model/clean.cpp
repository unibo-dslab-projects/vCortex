#include "vcortex/model/clean.hpp"

#include <algorithm>
#include <iterator>

namespace vcortex {

std::vector<Point> drop_transient(const std::vector<Point>& in, std::uint16_t min_scans) {
    std::vector<Point> out;
    // Most voxels of a mapped room are structure, so the output is close to the input size.
    out.reserve(in.size());
    std::copy_if(in.begin(), in.end(), std::back_inserter(out),
                 [min_scans](const Point& p) { return p.hits >= min_scans; });
    return out;
}

}  // namespace vcortex
