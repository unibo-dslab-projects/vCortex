#include "vcortex/model/floor.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <unordered_map>

namespace vcortex {

namespace {

// one map_builder voxel; finer bins split the floor layer
constexpr double kBin = 0.05;
// holds a floor tilted ~1 degree (+-5.5 cm on a 7.6 m room), leaves out most furniture feet
constexpr double kBand = 0.08;

// upside-down lidar sees the floor most, so the fullest bin is the floor; hash map, not
// array, so one far stray point cannot size it
double floor_guess(const std::vector<Point>& in) {
    std::unordered_map<long long, std::size_t> bins;
    for (const Point& p : in) {
        ++bins[static_cast<long long>(std::floor(p.z / kBin))];
    }
    long long best = 0;
    std::size_t best_count = 0;
    for (const auto& [bin, count] : bins) {
        if (count > best_count || (count == best_count && bin < best)) {
            best = bin;
            best_count = count;
        }
    }
    return (static_cast<double>(best) + 0.5) * kBin;
}

}  // namespace

Plane find_floor(const std::vector<Point>& in) {
    if (in.empty()) {
        throw std::runtime_error("find_floor: no points");
    }
    const double guess = floor_guess(in);

    // centred coordinates: c drops out, leaving a 2x2 system, and sums stay precise
    double mx = 0.0, my = 0.0, mz = 0.0;
    std::size_t n = 0;
    for (const Point& p : in) {
        if (std::abs(p.z - guess) <= kBand) {
            mx += p.x;
            my += p.y;
            mz += p.z;
            ++n;
        }
    }
    if (n < 3) {
        throw std::runtime_error("find_floor: fewer than 3 points near the floor height");
    }
    mx /= static_cast<double>(n);
    my /= static_cast<double>(n);
    mz /= static_cast<double>(n);

    double sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0;
    for (const Point& p : in) {
        if (std::abs(p.z - guess) <= kBand) {
            const double dx = p.x - mx, dy = p.y - my, dz = p.z - mz;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
            sxz += dx * dz;
            syz += dy * dz;
        }
    }
    // zero when floor points lie on one line (one-voxel corridor): tilt across it unknown
    const double det = sxx * syy - sxy * sxy;
    if (!(det > 1e-9 * sxx * syy)) {
        throw std::runtime_error("find_floor: floor points lie on a line, the plane is undefined");
    }

    Plane floor;
    floor.a = (sxz * syy - syz * sxy) / det;
    floor.b = (syz * sxx - sxz * sxy) / det;
    floor.c = mz - floor.a * mx - floor.b * my;
    return floor;
}

}  // namespace vcortex
