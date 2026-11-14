#include "vcortex/model/floor/floor.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <algorithm>
#include <iterator>
#include <map>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace vcortex {

namespace {

    // lowest dense layer per tile, so desks and chairs never vote; 0.5 m keeps 30+ points per tile
    constexpr double kTile = 0.5;
    constexpr std::size_t kTileMinPoints = 30;
    constexpr double kSeedPercentile = 0.05;
    // seeds farther than this from the fit are tiles with no floor seen (under a desk, a box)
    constexpr double kSeedResidual = 0.05;
    constexpr int kFitRounds = 10;
    // the 5th percentile sits under the noisy floor layer; the layer peak is the floor
    constexpr double kPeakRange = 0.20;
    constexpr double kPeakBin = 0.01;

    struct Tile {
        Eigen::Vector2d total = Eigen::Vector2d::Zero();
        std::vector<float> z;
    };

}  // namespace

/// Returns a vector containing the coordinates that better approximate/represent the floor tiles.
std::vector<Eigen::Vector3d> find_tiles_centroids(const std::vector<LidarPoint>& input_points) {
    std::map<std::pair<long long, long long>, Tile> tiles;

    for (const LidarPoint& p : input_points) {
        long long tile_x = static_cast<long long>(floor(p.position.x() / kTile));
        long long tile_y = static_cast<long long>(floor(p.position.y() / kTile));
        Tile& t = tiles[{tile_x, tile_y}];
        t.total += p.position.head<2>().cast<double>();
        t.z.push_back(p.position.z());
    }

    std::vector<Eigen::Vector3d> seeds;

    for (auto& tile : tiles) {
        Tile& t = tile.second;
        size_t len = t.z.size();
        if (len < kTileMinPoints) continue;
        const Eigen::Vector2d average = t.total / static_cast<double>(len);
        const auto z_index_percentile = static_cast<size_t>(static_cast<double>(len - 1) * kSeedPercentile);
        // z is in arrival order; a percentile needs it partly sorted
        std::nth_element(t.z.begin(), t.z.begin() + static_cast<std::ptrdiff_t>(z_index_percentile), t.z.end());
        seeds.emplace_back(average.x(), average.y(), t.z[z_index_percentile]);
    }

    return seeds;
}

/// Fits a plane from the given coordinates.
Plane fit_plane_from_coordinates(const std::vector<Eigen::Vector3d>& input_coordinates) {
    if (input_coordinates.size() < 3) {
        throw std::runtime_error("fit_plane_from_coordinates: fewer than 3 points");
    }
    const double n = static_cast<double>(input_coordinates.size());

    // finding the center of the plane
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& s : input_coordinates) mean += s;
    mean /= n;
    const double mx = mean.x(), my = mean.y(), mz = mean.z();

    // ax + by + c = z
    // a = sxz / sxx
    // b = syz / syy
    double sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0;
    for (const Eigen::Vector3d& s : input_coordinates) {
        const double dx = s.x() - mx;
        const double dy = s.y() - my;
        const double dz = s.z() - mz;
        sxy += dx * dy; // how much x and y are correlated
        sxx += dx * dx; // how many seeds are sparse in x
        syy += dy * dy; // how many seeds are sparse in y
        sxz += dx * dz; // how much the quota change with the x
        syz += dy * dz; // how much the quota change with the y
    }

    // det = sxx·syy − sxy²
    const double det = sxx * syy - sxy * sxy;
    if (!(det > 1e-9 * sxx * syy)) {
        throw std::runtime_error("fit_plane_from_seeds: seeds lie on a line, the plane is undefined");
    }

    // a = (sxz·syy − syz·sxy) / det
    // b = (syz·sxx − sxz·sxy) / det
    Plane plane;
    plane.a = (sxz * syy - syz * sxy) / det;
    plane.b = (syz * sxx - sxz * sxy) / det;
    plane.c = mz - plane.a * mx - plane.b * my;
    return plane;
}

/// Returns the peak shift between the given points and the given plane.
double find_peak_shift_between(const std::vector<LidarPoint>& input_points, const Plane& plane) {
    constexpr std::size_t kBins = static_cast<std::size_t>(2.0 * kPeakRange / kPeakBin + 0.5);
    std::vector<std::size_t> hist(kBins, 0);

    for (const LidarPoint& p : input_points) {
        const double r = height_above(plane, p);
        if (r < -kPeakRange || r >= kPeakRange) continue;
        const auto bin = static_cast<std::size_t>((r + kPeakRange) / kPeakBin);
        ++hist[std::min(bin, kBins - 1)];
    }

    const auto best_bin = std::max_element(hist.begin(), hist.end());
    if (*best_bin == 0) return 0.0;
    const auto peak_bin = static_cast<std::size_t>(std::distance(hist.begin(), best_bin));

    return (static_cast<double>(peak_bin) + 0.5) * kPeakBin - kPeakRange;
}

Plane find_floor(const std::vector<LidarPoint>& in) {
    if (in.empty()) {
        throw std::runtime_error("find_floor: no points");
    }
    std::vector<Eigen::Vector3d> centroids = find_tiles_centroids(in);
    Plane floor = fit_plane_from_coordinates(centroids);

    // removing the last centroids in order to avoid to fails due to the outliers
    for (int round = 0; round < kFitRounds; ++round) {
        const auto is_too_far = [&](const Eigen::Vector3d& s) {
            return std::abs(s.z() - (floor.a * s.x() + floor.b * s.y() + floor.c)) > kSeedResidual;
        };

        const auto last = std::remove_if(centroids.begin(), centroids.end(), is_too_far);
        if (last == centroids.end()) break;      // nothing dropped -> converged
        centroids.erase(last, centroids.end());
        floor = fit_plane_from_coordinates(centroids); // throws if too few centroids are left
    }

    floor.c += find_peak_shift_between(in, floor);
    return floor;
}

}  // namespace vcortex
