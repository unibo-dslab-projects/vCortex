#include "vcortex/model/align.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace vcortex {

namespace {

constexpr double kPi = 3.14159265358979323846;
// a wall one 5 cm cell thick falls into one bin when aligned
constexpr double kBin = 0.05;
// angles in tenths of a degree: integer steps, so folding into [0, 90) is exact
constexpr int kQuarter = 900;
constexpr int kCoarseStep = 5;
constexpr int kFineStep = 1;

double radians(int tenths) { return tenths * kPi / 1800.0; }

// sum of squared bin counts of x and y after turning by -theta: peaks when walls pile into
// few bins
double score(const std::vector<Vec2>& pts, double radius, int tenths,
             std::vector<std::uint32_t>& hx, std::vector<std::uint32_t>& hy) {
    const double c = std::cos(radians(tenths)), s = std::sin(radians(tenths));
    std::fill(hx.begin(), hx.end(), 0);
    std::fill(hy.begin(), hy.end(), 0);
    for (const Vec2& p : pts) {
        const double x = c * p.x + s * p.y;
        const double y = -s * p.x + c * p.y;
        ++hx[static_cast<std::size_t>((x + radius) / kBin)];
        ++hy[static_cast<std::size_t>((y + radius) / kBin)];
    }
    double sum = 0.0;
    for (const std::uint32_t n : hx) sum += static_cast<double>(n) * n;
    for (const std::uint32_t n : hy) sum += static_cast<double>(n) * n;
    return sum;
}

}  // namespace

// cell centres sit on the grid lattice, which pulls near-axis answers to 0: error up to
// atan(cell / wall length), 0.3 degrees on 8 m, less than one cell of drift
double dominant_yaw(const std::vector<Vec2>& band_cells) {
    if (band_cells.size() < 2) {
        return 0.0;
    }

    // centred, so every turned point stays within radius and the histograms have fixed size
    Vec2 mean;
    for (const Vec2& p : band_cells) {
        mean.x += p.x;
        mean.y += p.y;
    }
    mean.x /= static_cast<double>(band_cells.size());
    mean.y /= static_cast<double>(band_cells.size());
    std::vector<Vec2> pts;
    pts.reserve(band_cells.size());
    double radius = 0.0;
    for (const Vec2& p : band_cells) {
        pts.push_back({p.x - mean.x, p.y - mean.y});
        radius = std::max(radius, std::hypot(pts.back().x, pts.back().y));
    }
    const auto bins = static_cast<std::size_t>(2.0 * radius / kBin) + 2;
    std::vector<std::uint32_t> hx(bins), hy(bins);

    int best = 0;
    double best_score = -1.0;
    for (int t = 0; t < kQuarter; t += kCoarseStep) {
        const double sc = score(pts, radius, t, hx, hy);
        if (sc > best_score) {
            best_score = sc;
            best = t;
        }
    }
    const int coarse = best;
    for (int t = coarse - kCoarseStep; t <= coarse + kCoarseStep; t += kFineStep) {
        const double sc = score(pts, radius, t, hx, hy);
        if (sc > best_score) {
            best_score = sc;
            best = t;
        }
    }
    // the score repeats every 90 degrees, so -0.2 and 89.8 are the same answer
    return radians(((best % kQuarter) + kQuarter) % kQuarter);
}

}  // namespace vcortex
