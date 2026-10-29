#include "vcortex/model/candidates.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <tuple>

namespace vcortex {

namespace {

// metre limits become whole cells; the slack keeps 0.30 / 0.05 from rounding down to 5
constexpr double kSlack = 1e-9;

struct Run {
    int line = 0;             // grid row (Axis::Y) or column (Axis::X)
    int start = 0, last = 0;  // first and last occupied cell along the line
    std::size_t occupied = 0;
};

struct View {
    const Grid& grid;
    Axis axis;
    std::uint16_t min_column;

    int lines() const { return axis == Axis::Y ? grid.height : grid.width; }
    int along() const { return axis == Axis::Y ? grid.width : grid.height; }
    bool occupied(int line, int k) const {
        const int ix = axis == Axis::Y ? k : line;
        const int iy = axis == Axis::Y ? line : k;
        return grid.column[static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid.width) +
                           static_cast<std::size_t>(ix)] >= min_column;
    }
    double position(int line) const {
        return (axis == Axis::Y ? grid.origin.y : grid.origin.x) + (line + 0.5) * grid.cell;
    }
    double edge(int k) const {
        return (axis == Axis::Y ? grid.origin.x : grid.origin.y) + k * grid.cell;
    }
};

// short runs are cross-sections of perpendicular walls; kept, they would glue every line they
// touch into one candidate
std::vector<Run> find_runs(const View& v, int max_gap_cells, int min_piece_cells) {
    std::vector<Run> runs;
    for (int line = 0; line < v.lines(); ++line) {
        Run run;
        bool open = false;
        auto keep = [&] {
            if (run.last - run.start + 1 >= min_piece_cells) runs.push_back(run);
        };
        for (int k = 0; k < v.along(); ++k) {
            if (!v.occupied(line, k)) {
                continue;
            }
            if (open && k - run.last - 1 > max_gap_cells) {
                keep();
                open = false;
            }
            if (!open) {
                run = Run{line, k, k, 0};
                open = true;
            }
            run.last = k;
            ++run.occupied;
        }
        if (open) {
            keep();
        }
    }
    return runs;
}

int find_root(std::vector<int>& parent, int i) {
    while (parent[static_cast<std::size_t>(i)] != i) {
        i = parent[static_cast<std::size_t>(i)];
    }
    return i;
}

// neighbouring lines only, so a front one empty row away stays apart; spans may touch across
// a bridgeable gap, so a wall still slightly tilted, stepping between rows, stays whole
std::vector<Candidate> merge(const View& v, std::vector<Run> runs, int merge_lines,
                             int max_gap_cells, int min_cells) {
    std::sort(runs.begin(), runs.end(), [](const Run& a, const Run& b) {
        return std::tie(a.line, a.start) < std::tie(b.line, b.start);
    });
    std::vector<int> parent(runs.size());
    std::iota(parent.begin(), parent.end(), 0);
    for (std::size_t i = 0; i < runs.size(); ++i) {
        for (std::size_t j = i + 1;
             j < runs.size() && runs[j].line - runs[i].line <= merge_lines; ++j) {
            if (runs[j].line != runs[i].line &&
                runs[j].start <= runs[i].last + max_gap_cells + 1 &&
                runs[i].start <= runs[j].last + max_gap_cells + 1) {
                parent[static_cast<std::size_t>(find_root(parent, static_cast<int>(j)))] =
                    find_root(parent, static_cast<int>(i));
            }
        }
    }

    std::vector<std::vector<const Run*>> groups(runs.size());
    for (std::size_t i = 0; i < runs.size(); ++i) {
        groups[static_cast<std::size_t>(find_root(parent, static_cast<int>(i)))].push_back(
            &runs[i]);
    }
    std::vector<Candidate> out;
    for (const auto& group : groups) {
        if (group.empty()) {
            continue;
        }
        int start = group.front()->start, last = group.front()->last;
        double weighted = 0.0;
        std::size_t total = 0;
        for (const Run* r : group) {
            start = std::min(start, r->start);
            last = std::max(last, r->last);
            weighted += v.position(r->line) * static_cast<double>(r->occupied);
            total += r->occupied;
        }
        // length after merging: each row piece of a stepping wall can be under min_length
        if (last - start + 1 < min_cells) {
            continue;
        }
        // union over lines, so a wall seen on two rows counts each cell along it once
        std::vector<bool> seen(static_cast<std::size_t>(last - start + 1), false);
        for (const Run* r : group) {
            for (int k = r->start; k <= r->last; ++k) {
                if (v.occupied(r->line, k)) {
                    seen[static_cast<std::size_t>(k - start)] = true;
                }
            }
        }
        Candidate c;
        c.axis = v.axis;
        c.position = weighted / static_cast<double>(total);
        c.from = v.edge(start);
        c.to = v.edge(last + 1);
        c.cells = static_cast<std::size_t>(std::count(seen.begin(), seen.end(), true));
        out.push_back(c);
    }
    return out;
}

}  // namespace

std::vector<Candidate> find_candidates(const Grid& grid, const CandidateParams& p) {
    const int max_gap_cells = static_cast<int>(std::floor(p.max_gap / grid.cell + kSlack));
    const int min_cells = static_cast<int>(std::ceil(p.min_length / grid.cell - kSlack));
    // "closer than": two lines exactly merge_distance apart stay separate
    const int merge_lines = static_cast<int>(std::ceil(p.merge_distance / grid.cell - kSlack)) - 1;

    std::vector<Candidate> out;
    for (const Axis axis : {Axis::X, Axis::Y}) {
        const View v{grid, axis, p.min_column};
        std::vector<Candidate> part = merge(v, find_runs(v, max_gap_cells, (min_cells + 1) / 2),
                                           merge_lines, max_gap_cells, min_cells);
        std::sort(part.begin(), part.end(), [](const Candidate& a, const Candidate& b) {
            return std::tie(a.position, a.from) < std::tie(b.position, b.from);
        });
        out.insert(out.end(), part.begin(), part.end());
    }
    return out;
}

}  // namespace vcortex
