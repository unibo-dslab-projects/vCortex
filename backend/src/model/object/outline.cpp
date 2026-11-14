#include "vcortex/model/object/outline.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "vcortex/geometry/grids.hpp"

namespace vcortex {

namespace {

// listed counter-clockwise, so the next value is a left turn
enum class Direction { east, north, west, south };

Direction turn_left(Direction direction) {
    return static_cast<Direction>((static_cast<int>(direction) + 1) % 4);
}

struct BoundaryEdge {
    Eigen::Vector2i start;
    Direction direction = Direction::east;
};

Eigen::Vector2i end_of(const BoundaryEdge& edge) {
    switch (edge.direction) {
        case Direction::east: return {edge.start.x() + 1, edge.start.y()};
        case Direction::north: return {edge.start.x(), edge.start.y() + 1};
        case Direction::west: return {edge.start.x() - 1, edge.start.y()};
        case Direction::south: return {edge.start.x(), edge.start.y() - 1};
    }
    return edge.start;
}

/// Finds the boundary edges of an object.
std::vector<BoundaryEdge> find_boundary_edges(const ObjectGrid& grid,
                                              const std::vector<std::uint32_t>& labels,
                                              std::uint32_t object_id) {
    const auto is_in_object = [&](int column, int row) {
        if (column < 0 || column >= grid.width || row < 0 || row >= grid.height) return false;
        return labels[static_cast<std::size_t>(row * grid.width + column)] == object_id;
    };

    std::vector<BoundaryEdge> edges;
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            if (!is_in_object(column, row)) continue;

            if (!is_in_object(column, row - 1)) edges.push_back({{column, row}, Direction::east});
            if (!is_in_object(column + 1, row)) edges.push_back({{column + 1, row}, Direction::north});
            if (!is_in_object(column, row + 1)) edges.push_back({{column + 1, row + 1}, Direction::west});
            if (!is_in_object(column - 1, row)) edges.push_back({{column, row + 1}, Direction::south});
        }
    }
    return edges;
}

std::vector<std::vector<BoundaryEdge>> chain_rings(const ObjectGrid& grid,
                                                   const std::vector<BoundaryEdge>& edges) {
    const auto corner_key = [&](Eigen::Vector2i corner) {
        return static_cast<std::size_t>(corner.y()) * static_cast<std::size_t>(grid.width + 1) +
               static_cast<std::size_t>(corner.x());
    };

    std::unordered_map<std::size_t, std::vector<std::size_t>> edges_starting_at;
    for (std::size_t index = 0; index < edges.size(); ++index) {
        edges_starting_at[corner_key(edges[index].start)].push_back(index);
    }

    std::vector<bool> is_used(edges.size(), false);
    std::vector<std::vector<BoundaryEdge>> rings;
    for (std::size_t first_index = 0; first_index < edges.size(); ++first_index) {
        if (is_used[first_index]) continue;

        std::vector<BoundaryEdge> ring;
        std::optional<std::size_t> current_index = first_index;
        while (current_index) {
            const BoundaryEdge& current = edges[*current_index];
            is_used[*current_index] = true;
            ring.push_back(current);

            current_index.reset();
            for (const std::size_t candidate_index : edges_starting_at[corner_key(end_of(current))]) {
                if (is_used[candidate_index]) continue;
                // turn left if more than one is found
                if (!current_index || edges[candidate_index].direction == turn_left(current.direction)) {
                    current_index = candidate_index;
                }
            }
        }
        rings.push_back(std::move(ring));
    }
    return rings;
}

// corners-only compression of all the edges of a ring
std::vector<Eigen::Vector2i> keep_corners_only(const std::vector<BoundaryEdge>& ring) {
    std::vector<Eigen::Vector2i> corners;
    for (std::size_t index = 0; index < ring.size(); ++index) {
        const BoundaryEdge& previous = ring[(index + ring.size() - 1) % ring.size()];
        if (ring[index].direction != previous.direction) corners.push_back(ring[index].start);
    }
    return corners;
}

// a ring of right angles cannot have fewer
constexpr std::size_t MIN_RING_CORNERS = 4;

Direction direction_between(Eigen::Vector2i from, Eigen::Vector2i to) {
    if (to.x() > from.x()) return Direction::east;
    if (to.x() < from.x()) return Direction::west;
    if (to.y() > from.y()) return Direction::north;
    return Direction::south;
}

bool is_horizontal(Direction direction) {
    return direction == Direction::east || direction == Direction::west;
}

/// Returns a bool indicating if moving `moved_corner` to `corner` would reverse the direction of `neighbour`.
bool would_reverse(const Eigen::Vector2i& corner, const Eigen::Vector2i& moved_corner,
                   const Eigen::Vector2i& neighbour) {
    if (moved_corner == neighbour) return false;

    return direction_between(corner, neighbour) != direction_between(moved_corner, neighbour);
}

// remove the points that are on a straight line, leaving only the corners
void remove_straight_corners(std::vector<Eigen::Vector2i>& ring) {
    bool has_changed = true;
    while (has_changed && ring.size() > 2) {
        has_changed = false;
        for (std::size_t index = 0; index < ring.size(); ++index) {
            const Eigen::Vector2i previous = ring[(index + ring.size() - 1) % ring.size()];
            const Eigen::Vector2i corner = ring[index];
            const Eigen::Vector2i next = ring[(index + 1) % ring.size()];
            const bool is_repeated = previous == corner;
            const bool are_on_straight_line = (previous.x() == corner.x() && corner.x() == next.x()) ||
                                             (previous.y() == corner.y() && corner.y() == next.y());
            if (!is_repeated && !are_on_straight_line) continue;

            ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(index));
            has_changed = true;
            break;
        }
    }
}

/// Straightens angles in a corner by straightening the lines composing that corner
std::optional<std::vector<Eigen::Vector2i>> straighten_corners(const std::vector<Eigen::Vector2i>& ring,
                                                       std::size_t corner_index) {
    const std::size_t count = ring.size();
    const std::size_t index_a = (corner_index + count - 1) % count;
    const std::size_t index_b = corner_index;
    const std::size_t index_c = (corner_index + 1) % count;
    const std::size_t index_d = (corner_index + 2) % count;
    const Eigen::Vector2i a = ring[index_a], b = ring[index_b], c = ring[index_c], d = ring[index_d];
    const bool runs_horizontally = is_horizontal(direction_between(a, b));

    // choosing a dominant side (the longer cause it should be more precise)
    std::vector<Eigen::Vector2i> straightened = ring;
    if ((b - a).lpNorm<1>() >= (d - c).lpNorm<1>()) {
        const Eigen::Vector2i moved_d =
            runs_horizontally ? Eigen::Vector2i{d.x(), a.y()} : Eigen::Vector2i{a.x(), d.y()};
        if (would_reverse(d, moved_d, ring[(corner_index + 3) % count])) return std::nullopt;

        straightened[index_d] = moved_d;
    } else {
        const Eigen::Vector2i moved_a =
            runs_horizontally ? Eigen::Vector2i{a.x(), d.y()} : Eigen::Vector2i{d.x(), a.y()};
        if (would_reverse(a, moved_a, ring[(corner_index + count - 2) % count])) return std::nullopt;

        straightened[index_a] = moved_a;
    }

    // getting the indexes that were making the angle to straighten
    const std::size_t later_index = std::max(index_b, index_c);
    const std::size_t earlier_index = std::min(index_b, index_c);
    straightened.erase(straightened.begin() + static_cast<std::ptrdiff_t>(later_index));
    straightened.erase(straightened.begin() + static_cast<std::ptrdiff_t>(earlier_index));
    remove_straight_corners(straightened);
    return straightened;
}

}  // namespace

std::vector<Eigen::Vector2i> trace_outer_ring(const ObjectGrid& grid,
                                              const std::vector<std::uint32_t>& labels,
                                              std::uint32_t object_id) {
    std::vector<Eigen::Vector2i> outer_ring;
    long long outer_doubled_area = 0;
    // holes run clockwise, so their area is negative and they never win; the largest ring guards
    // pieces that are not 4-connected, and a tie keeps the ring found first
    const std::vector<BoundaryEdge> edges = find_boundary_edges(grid, labels, object_id);
    for (const std::vector<BoundaryEdge>& ring : chain_rings(grid, edges)) {
        std::vector<Eigen::Vector2i> corners = keep_corners_only(ring);
        const long long doubled_area = doubled_signed_area(corners);
        if (doubled_area <= outer_doubled_area) continue;

        outer_doubled_area = doubled_area;
        outer_ring = std::move(corners);
    }
    return outer_ring;
}

void drop_small_segments(std::vector<Eigen::Vector2i>& ring, int min_segment_cells) {
    while (ring.size() > MIN_RING_CORNERS) {
        const std::size_t count = ring.size();
        std::optional<std::vector<Eigen::Vector2i>> best_straightened;
        // the threshold and the shortest step so far; strict < keeps the first one on a tie
        int best_segment_length = min_segment_cells;
        for (std::size_t index = 0; index < count; ++index) {
            const Eigen::Vector2i a = ring[(index + count - 1) % count];
            const Eigen::Vector2i b = ring[index];
            const Eigen::Vector2i c = ring[(index + 1) % count];
            const Eigen::Vector2i d = ring[(index + 2) % count];
            const int segment_length = (c - b).lpNorm<1>();
            if (segment_length >= best_segment_length) continue;
            if (direction_between(a, b) != direction_between(c, d)) continue;

            std::optional<std::vector<Eigen::Vector2i>> straightened = straighten_corners(ring, index);
            if (!straightened) continue;

            best_segment_length = segment_length;
            best_straightened = std::move(straightened);
        }
        if (!best_straightened) return;

        ring = std::move(*best_straightened);
    }
}

}  // namespace vcortex
