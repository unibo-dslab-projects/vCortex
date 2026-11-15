#include "vcortex/model/wall/candidates.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <tuple>

namespace vcortex {

namespace {

// metre limits become whole cells; the slack keeps 0.30 / 0.05 from rounding down to 5
constexpr double kSlack = 1e-9;

/// Occupied cells on one row or column, with the bridged gaps between them.
struct OccupiedSpan {
    int line = 0;              // grid row (Axis::Y) or column (Axis::X)
    int start = 0, last = 0;   // first and last occupied cell along the line
    std::size_t occupied = 0;  // cells really occupied; last - start + 1 also counts the gaps
};

// reads the grid by rows (Axis::Y) or by columns (Axis::X), so the span and merge code is
// written once for both directions
struct View {
    const Grid& grid;
    Axis axis;
    std::uint16_t min_column;  // points per cell to call it occupied

    int lines() const { return axis == Axis::Y ? grid.height : grid.width; }
    int along() const { return axis == Axis::Y ? grid.width : grid.height; }
    /// True when cell k of the line holds at least min_column points.
    bool occupied(int line, int k) const {
        const int ix = axis == Axis::Y ? k : line;
        const int iy = axis == Axis::Y ? line : k;
        return grid.column[static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid.width) +
                           static_cast<std::size_t>(ix)] >= min_column;
    }
    /// Metres to the middle of the line, across it.
    double position(int line) const {
        return (axis == Axis::Y ? grid.origin.y() : grid.origin.x()) + (line + 0.5) * grid.cell;
    }
    /// Metres to the lower edge of cell k, along the line.
    double edge(int k) const {
        return (axis == Axis::Y ? grid.origin.x() : grid.origin.y()) + k * grid.cell;
    }
};

/// Cuts every row or column into spans of occupied cells, bridging gaps up to max_gap_cells and
/// dropping spans shorter than min_piece_cells.
// short spans are cross-sections of perpendicular walls; kept, they would glue every line they
// touch into one candidate
std::vector<OccupiedSpan> find_occupied_spans(const View& v, int max_gap_cells,
                                              int min_piece_cells) {
    std::vector<OccupiedSpan> spans;
    for (int line = 0; line < v.lines(); ++line) {
        OccupiedSpan span;
        bool open = false;
        auto keep = [&] {
            if (span.last - span.start + 1 >= min_piece_cells) spans.push_back(span);
        };
        for (int k = 0; k < v.along(); ++k) {
            if (!v.occupied(line, k)) {
                continue;
            }
            if (open && k - span.last - 1 > max_gap_cells) {
                keep();
                open = false;
            }
            if (!open) {
                span = OccupiedSpan{line, k, k, 0};
                open = true;
            }
            span.last = k;
            ++span.occupied;
        }
        if (open) {
            keep();
        }
    }
    return spans;
}

/// Union-find: follows the parent links from span i up to the root that names its group.
int find_root(std::vector<int>& parent, int i) {
    while (parent[static_cast<std::size_t>(i)] != i) {
        i = parent[static_cast<std::size_t>(i)];
    }
    return i;
}

/// Joins spans on nearby lines that touch along the line into candidates, dropping those
/// shorter than min_cells.
// neighbouring lines only, so a front one empty row away stays apart; spans may touch across
// a bridgeable gap, so a wall still slightly tilted, stepping between rows, stays whole
std::vector<Candidate> merge(const View& v, std::vector<OccupiedSpan> spans, int merge_lines,
                             int max_gap_cells, int min_cells) {
    // sorted by line, the inner loop stops at the first span more than merge_lines away
    std::sort(spans.begin(), spans.end(), [](const OccupiedSpan& a, const OccupiedSpan& b) {
        return std::tie(a.line, a.start) < std::tie(b.line, b.start);
    });
    // every span starts as its own group, named by its own index (union-find)
    std::vector<int> parent(spans.size());
    std::iota(parent.begin(), parent.end(), 0);
    // attach nearby spans to the same group, so they merge together
    for (std::size_t i = 0; i < spans.size(); ++i) {
        for (std::size_t j = i + 1;
             j < spans.size() && spans[j].line - spans[i].line <= merge_lines; ++j) {
            // spans of one line were cut apart on purpose by a gap wider than max_gap
            if (spans[j].line != spans[i].line &&
                spans[j].start <= spans[i].last + max_gap_cells + 1 &&
                spans[i].start <= spans[j].last + max_gap_cells + 1) {
                parent[static_cast<std::size_t>(find_root(parent, static_cast<int>(j)))] =
                    find_root(parent, static_cast<int>(i));
            }
        }
    }

    std::vector<std::vector<const OccupiedSpan*>> groups(spans.size());
    for (std::size_t i = 0; i < spans.size(); ++i) {
        groups[static_cast<std::size_t>(find_root(parent, static_cast<int>(i)))].push_back(
            &spans[i]);
    }
    std::vector<Candidate> out;
    for (const auto& group : groups) {
        if (group.empty()) {
            continue;
        }
        int start = group.front()->start, last = group.front()->last;
        // position = line middles weighted by occupied cells, so a few stray cells on a
        // neighbour row barely move the wall
        double weighted = 0.0;
        std::size_t total = 0;
        for (const OccupiedSpan* span : group) {
            start = std::min(start, span->start);
            last = std::max(last, span->last);
            weighted += v.position(span->line) * static_cast<double>(span->occupied);
            total += span->occupied;
        }
        // length after merging: each row piece of a stepping wall can be under min_length
        // a shorter span can be a perpendicular candidate wall
        if (last - start + 1 < min_cells) {
            continue;
        }
        // union over lines, so a wall seen on two rows counts each cell along it once
        std::vector<bool> seen(static_cast<std::size_t>(last - start + 1), false);
        for (const OccupiedSpan* span : group) {
            for (int k = span->start; k <= span->last; ++k) {
                if (v.occupied(span->line, k)) {
                    seen[static_cast<std::size_t>(k - start)] = true;
                }
            }
        }
        Candidate c;
        c.axis = v.axis;
        c.position = weighted / static_cast<double>(total);
        c.from = v.edge(start);
        c.to = v.edge(last + 1);  // far edge of the last cell
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
        // a wall stepping across two rows splits in two, each piece at least half its length
        std::vector<Candidate> part =
            merge(v, find_occupied_spans(v, max_gap_cells, (min_cells + 1) / 2), merge_lines,
                  max_gap_cells, min_cells);
        std::sort(part.begin(), part.end(), [](const Candidate& a, const Candidate& b) {
            return std::tie(a.position, a.from) < std::tie(b.position, b.from);
        });
        out.insert(out.end(), part.begin(), part.end());
    }
    return out;
}

}  // namespace vcortex
