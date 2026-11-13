#include "vcortex/model/floor/floor.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <utility>
#include <vector>


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

    struct Seed {
        double avg_x, avg_y, percentile_z;
    };

    struct Tile {
        double total_x, total_y;
        std::vector<float> z;
    };

}  // namespace

std::vector<Seed> tile_seeds(const std::vector<Point>& input_points) {
    std::map<std::pair<long long, long long>, Tile> tiles;

    for (const Point& p : input_points) {
        long long tile_x = static_cast<long long>(floor(p.x / kTile));
        long long tile_y = static_cast<long long>(floor(p.y / kTile));
        Tile& t = tiles[{tile_x, tile_y}];
        t.total_x += p.x;
        t.total_y += p.y;
        t.z.push_back(p.z);
    }

    std::vector<Seed> seeds;

    for (auto& tile : tiles) {
        Tile& t = tile.second;
        size_t len = t.z.size();
        if (len < kTileMinPoints) continue;
        double avg_x = t.total_x / static_cast<double>(len);
        double avg_y = t.total_y / static_cast<double>(len);
        const auto z_index_percentile = static_cast<size_t>(static_cast<double>(len - 1) * kSeedPercentile);
        // z is in arrival order; a percentile needs it partly sorted
        std::nth_element(t.z.begin(), t.z.begin() + static_cast<std::ptrdiff_t>(z_index_percentile), t.z.end());
        seeds.push_back(Seed { avg_x, avg_y, t.z[z_index_percentile] });
    }

    return seeds;
}

Plane fit_plane_from_seeds(const std::vector<Seed>& seeds) {
    if (seeds.size() < 3) {
        throw std::runtime_error("fit_plane_from_seeds: fewer than 3 seeds");
    }
    const double n = static_cast<double>(seeds.size());

    // finding the center of the plane
    double mx = 0.0, my = 0.0, mz = 0.0;
    for (const Seed& s : seeds) {
        mx += s.avg_x;
        my += s.avg_y;
        mz += s.percentile_z;
    }
    mx /= n;
    my /= n;
    mz /= n;


    // ax + by + c = z
    // a = sxz / sxx
    // b = syz / syy
    double sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0;
    for (const Seed& s : seeds) {
        const double dx = s.avg_x - mx;
        const double dy = s.avg_y - my;
        const double dz = s.percentile_z - mz;
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

double find_peak_shift(const std::vector<Point>& input_points, const Plane& plane) {
    constexpr std::size_t kBins = static_cast<std::size_t>(2.0 * kPeakRange / kPeakBin + 0.5);
    std::vector<std::size_t> hist(kBins, 0);

    for (const Point& p : input_points) {
        const double r = p.z - (plane.a * p.x + plane.b * p.y + plane.c);
        if (r < -kPeakRange || r >= kPeakRange) continue;
        const auto bin = static_cast<std::size_t>((r + kPeakRange) / kPeakBin);
        ++hist[std::min(bin, kBins - 1)];
    }

    std::size_t best = 0;
    for (std::size_t i = 1; i < kBins; ++i) {
        if (hist[i] > hist[best]) best = i;
    }
    if (hist[best] == 0) return 0.0;

    return (static_cast<double>(best) + 0.5) * kPeakBin - kPeakRange;
}

Plane find_floor(const std::vector<Point>& in) {
    if (in.empty()) {
        throw std::runtime_error("find_floor: no points");
    }
    std::vector<Seed> seeds = tile_seeds(in);
    Plane floor = fit_plane_from_seeds(seeds);

    // removing the last seeds in order to avoid to fails due to the outliers
    for (int round = 0; round < kFitRounds; ++round) {
        const auto is_too_far = [&](const Seed& s) {
            return std::abs(s.percentile_z - (floor.a * s.avg_x + floor.b * s.avg_y + floor.c)) > kSeedResidual;
        };

        const auto last = std::remove_if(seeds.begin(), seeds.end(), is_too_far);
        if (last == seeds.end()) break;      // nothing dropped -> converged
        seeds.erase(last, seeds.end());
        floor = fit_plane_from_seeds(seeds); // throws if too few seeds are left
    }

    floor.c += find_peak_shift(in, floor);
    return floor;
}

}  // namespace vcortex
