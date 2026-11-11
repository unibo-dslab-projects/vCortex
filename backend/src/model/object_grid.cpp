#include "vcortex/model/object_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "vcortex/geometry/points.hpp"

namespace vcortex {

namespace {

bool has_neighbour_with_value(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                              int row, int column, int radius_cells, std::uint8_t value) {
    for (int row_offset = -radius_cells; row_offset <= radius_cells; ++row_offset) {
        for (int column_offset = -radius_cells; column_offset <= radius_cells; ++column_offset) {
            const int neighbour_row = row + row_offset;
            const int neighbour_column = column + column_offset;
            if (neighbour_row < 0 || neighbour_row >= grid.height || neighbour_column < 0 ||
                neighbour_column >= grid.width) {
                continue;
            }
            const auto index =
                static_cast<std::size_t>(neighbour_row * grid.width + neighbour_column);
            if (mask[index] == value) return true;
        }
    }
    return false;
}

// only the 4 side neighbours inside the grid, so cells touching at a corner stay apart
std::vector<std::size_t> neighbour_indices(const ObjectGrid& grid, std::size_t index) {
    const int row = static_cast<int>(index) / grid.width;
    const int column = static_cast<int>(index) % grid.width;
    std::vector<std::size_t> neighbours;
    const int steps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& step : steps) {
        const int neighbour_row = row + step[0];
        const int neighbour_column = column + step[1];
        if (neighbour_row < 0 || neighbour_row >= grid.height || neighbour_column < 0 ||
            neighbour_column >= grid.width) {
            continue;
        }
        neighbours.push_back(static_cast<std::size_t>(neighbour_row * grid.width + neighbour_column));
    }
    return neighbours;
}

}  // namespace

ObjectGrid build_object_grid(const std::vector<Point>& furniture_points, const Plane& floor,
                             double cell, int padding_cells, int cell_min_points) {
    ObjectGrid grid;
    grid.cell = cell;
    if (furniture_points.empty()) return grid;

    const Bounds bounds = point_bounds(furniture_points);

    // adding padding to support spatial algorithms boundary conditions
    const double padding_metres = padding_cells * cell;
    grid.origin = {bounds.min_x - padding_metres, bounds.min_y - padding_metres};
    grid.width =
        static_cast<int>(std::floor((bounds.max_x - bounds.min_x) / cell)) + 1 + 2 * padding_cells;
    grid.height = static_cast<int>(std::floor((bounds.max_y - bounds.min_y) / cell)) +
                  1 + 2 * padding_cells;

    const auto cell_count =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    grid.is_occupied.assign(cell_count, 0);
    grid.point_count.assign(cell_count, 0);
    grid.top_metres.assign(cell_count, 0.0F);
    grid.cell_of_point.reserve(furniture_points.size());

    // voxeling the points cloud in a 2D grid, counting points per cell
    for (const Point& point : furniture_points) {
        // clamped: rounding can push a point on the max edge one cell past the data
        const int column = std::clamp(static_cast<int>(std::floor((point.x - grid.origin.x) / cell)),
                                      0, grid.width - 1);
        const int row = std::clamp(static_cast<int>(std::floor((point.y - grid.origin.y) / cell)),
                                   0, grid.height - 1);
        const std::size_t index = static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
                                  static_cast<std::size_t>(column);
        ++grid.point_count[index];
        grid.top_metres[index] =
            std::max(grid.top_metres[index], static_cast<float>(height_above(floor, point)));
        grid.cell_of_point.push_back(index);
    }

    for (std::size_t index = 0; index < cell_count; ++index) {
        grid.is_occupied[index] =
            grid.point_count[index] >= static_cast<std::uint32_t>(cell_min_points) ? 1 : 0;
    }
    return grid;
}

std::vector<std::uint8_t> grow_by_cell(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                                 int radius_cells) {
    std::vector<std::uint8_t> result(mask.size(), 0);
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const bool is_full = has_neighbour_with_value(grid, mask, row, column, radius_cells, 1);
            result[static_cast<std::size_t>(row * grid.width + column)] = is_full;
        }
    }
    return result;
}

std::vector<std::uint8_t> shrink_by_cell(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask,
                                int radius_cells) {
    std::vector<std::uint8_t> result(mask.size(), 0);
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const bool is_full = !has_neighbour_with_value(grid, mask, row, column, radius_cells, 0);
            result[static_cast<std::size_t>(row * grid.width + column)] = is_full;
        }
    }
    return result;
}

std::vector<std::uint32_t> label_objects(const ObjectGrid& grid, const std::vector<std::uint8_t>& mask) {
    std::vector<std::uint32_t> result(mask.size(), 0);

    std::uint32_t object_id = 0;
    for (int row = 0; row < grid.height; ++row) {
        for (int column = 0; column < grid.width; ++column) {
            const std::size_t index = static_cast<std::size_t>(row * grid.width + column);
            if (mask[index] && !result[index]) {
                result[index] = ++object_id;
                // bfs
                std::queue<std::pair<int, int>> to_visit;
                to_visit.push({row, column});
                while (!to_visit.empty()) {
                    const auto [cell_row, cell_column] = to_visit.front();
                    to_visit.pop();
                    const int steps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                    for (const auto& step : steps) {
                        const int neighbour_row = cell_row + step[0];
                        const int neighbour_column = cell_column + step[1];
                        if (neighbour_row < 0 || neighbour_row >= grid.height ||
                            neighbour_column < 0 || neighbour_column >= grid.width) {
                            continue;
                        }
                        const auto neighbour_index =
                            static_cast<std::size_t>(neighbour_row * grid.width + neighbour_column);
                        if (!mask[neighbour_index] || result[neighbour_index]) continue;
                        result[neighbour_index] = object_id;
                        to_visit.push({neighbour_row, neighbour_column});
                    }
                }
            }
        }
    }
    return result;
}


std::vector<std::uint32_t> assign_non_labelled_objects(const ObjectGrid& grid, const std::vector<std::uint32_t>& mask, const std::vector<std::uint8_t>& occupied) {
    std::vector<std::uint32_t> result(mask.size(), 0);
    if (mask.empty()) return result;
    std::queue<std::pair<std::size_t, std::uint32_t>> to_visit;
    std::uint32_t max_object_id = *std::max_element(mask.begin(), mask.end());

    for (std::size_t idx = 0; idx < mask.size(); ++idx) {
        result[idx] = mask[idx];
        if (mask[idx]) {
            to_visit.push({idx, mask[idx]});
        }
    }

    const int column_size = grid.width;
    const int row_size = grid.height;

    std::size_t next_leftover = 0;
    while (true) {
        // occupied points non-labelled by the core algorithm are assigned a new label
        if (to_visit.empty()) {
            while (next_leftover < mask.size() &&
                   (!occupied[next_leftover] || result[next_leftover])) {
                ++next_leftover;
            }
            if (next_leftover == mask.size()) break;
            result[next_leftover] = ++max_object_id;
            to_visit.push({next_leftover, max_object_id});
        }
        const auto [cell_index, cell_object_id] = to_visit.front();
        to_visit.pop();
        const int steps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& step : steps) {
            const int neighbour_row = static_cast<int>(cell_index) / column_size + step[0];
            const int neighbour_column = static_cast<int>(cell_index) % column_size + step[1];
            if (neighbour_row < 0 || neighbour_row >= row_size ||
                neighbour_column < 0 || neighbour_column >= column_size) {
              continue;
            }
            const auto neighbour_index =
                static_cast<std::size_t>(neighbour_row * grid.width + neighbour_column);
            if (result[neighbour_index] || !occupied[neighbour_index]) continue;
            result[neighbour_index] = cell_object_id;
            to_visit.push({neighbour_index, cell_object_id});
        }
    }

    return result;
}

std::vector<std::uint32_t> split_by_height(const ObjectGrid& grid,
                                           const std::vector<std::uint32_t>& labels,
                                           double height_step, int min_piece_cells) {
    std::vector<std::uint32_t> result(labels.size(), 0);
    std::vector<std::vector<std::size_t>> cells_of_region(1);  // region 0 means no region
    std::vector<double> top_sum_of_region(1, 0.0);

    // grow regions inside each piece; only cells with points have a top, the cells closing
    // filled in are given back at the end
    for (std::size_t index = 0; index < labels.size(); ++index) {
        if (!labels[index] || !grid.point_count[index] || result[index]) continue;
        const auto region_id = static_cast<std::uint32_t>(cells_of_region.size());
        cells_of_region.emplace_back();
        top_sum_of_region.push_back(0.0);
        result[index] = region_id;
        std::queue<std::size_t> to_visit;
        to_visit.push(index);
        while (!to_visit.empty()) {
            const std::size_t cell_index = to_visit.front();
            to_visit.pop();
            cells_of_region[region_id].push_back(cell_index);
            top_sum_of_region[region_id] += grid.top_metres[cell_index];
            for (const std::size_t neighbour_index : neighbour_indices(grid, cell_index)) {
                if (result[neighbour_index] || !grid.point_count[neighbour_index] ||
                    labels[neighbour_index] != labels[cell_index]) {
                    continue;
                }
                const double top_jump =
                    std::abs(grid.top_metres[neighbour_index] - grid.top_metres[cell_index]);
                if (top_jump > height_step) continue;
                result[neighbour_index] = region_id;
                to_visit.push(neighbour_index);
            }
        }
    }

    // cell tops are noisy, so a small region is mostly noise: it joins the neighbour region of
    // the same piece with the closest mean top
    const auto mean_top = [&](std::uint32_t region_id) {
        return top_sum_of_region[region_id] /
               static_cast<double>(cells_of_region[region_id].size());
    };
    for (std::uint32_t region_id = 1; region_id < cells_of_region.size(); ++region_id) {
        std::vector<std::size_t>& cells = cells_of_region[region_id];
        if (cells.empty() || cells.size() >= static_cast<std::size_t>(min_piece_cells)) continue;

        std::uint32_t closest_region_id = 0;
        double closest_top_difference = std::numeric_limits<double>::infinity();
        for (const std::size_t cell_index : cells) {
            for (const std::size_t neighbour_index : neighbour_indices(grid, cell_index)) {
                const std::uint32_t neighbour_region_id = result[neighbour_index];
                if (!neighbour_region_id || neighbour_region_id == region_id ||
                    labels[neighbour_index] != labels[cell_index]) {
                    continue;
                }
                const double top_difference =
                    std::abs(mean_top(neighbour_region_id) - mean_top(region_id));
                if (top_difference < closest_top_difference) {
                    closest_top_difference = top_difference;
                    closest_region_id = neighbour_region_id;
                }
            }
        }
        // no neighbour region: the whole piece is this small region, so it stays as it is
        if (!closest_region_id) continue;

        for (const std::size_t cell_index : cells) result[cell_index] = closest_region_id;
        std::vector<std::size_t>& closest_cells = cells_of_region[closest_region_id];
        closest_cells.insert(closest_cells.end(), cells.begin(), cells.end());
        top_sum_of_region[closest_region_id] += top_sum_of_region[region_id];
        cells.clear();
        top_sum_of_region[region_id] = 0.0;
    }

    std::vector<std::uint8_t> is_in_piece(labels.size(), 0);
    for (std::size_t index = 0; index < labels.size(); ++index) {
        is_in_piece[index] = labels[index] ? 1 : 0;
    }
    return assign_non_labelled_objects(grid, result, is_in_piece);
}

namespace {

// listed counter-clockwise, so the next value is a left turn
enum class Direction { east, north, west, south };

Direction turn_left(Direction direction) {
    return static_cast<Direction>((static_cast<int>(direction) + 1) % 4);
}

struct BoundaryEdge {
    GridCorner start;
    Direction direction = Direction::east;
};

GridCorner end_of(const BoundaryEdge& edge) {
    switch (edge.direction) {
        case Direction::east: return {edge.start.x + 1, edge.start.y};
        case Direction::north: return {edge.start.x, edge.start.y + 1};
        case Direction::west: return {edge.start.x - 1, edge.start.y};
        case Direction::south: return {edge.start.x, edge.start.y - 1};
    }
    return edge.start;
}

/// finds the boundary edges af an object
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
    const auto corner_key = [&](GridCorner corner) {
        return static_cast<std::size_t>(corner.y) * static_cast<std::size_t>(grid.width + 1) +
               static_cast<std::size_t>(corner.x);
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

// a corner stays only where the direction turns
std::vector<GridCorner> keep_corners_only(const std::vector<BoundaryEdge>& ring) {
    std::vector<GridCorner> corners;
    for (std::size_t index = 0; index < ring.size(); ++index) {
        const BoundaryEdge& previous = ring[(index + ring.size() - 1) % ring.size()];
        if (ring[index].direction != previous.direction) corners.push_back(ring[index].start);
    }
    return corners;
}

// shoelace sum, doubled to stay in integers: positive for a counter-clockwise ring
long long doubled_signed_area(const std::vector<GridCorner>& ring) {
    long long doubled_area = 0;
    for (std::size_t index = 0; index < ring.size(); ++index) {
        const GridCorner& current = ring[index];
        const GridCorner& next = ring[(index + 1) % ring.size()];
        doubled_area += static_cast<long long>(current.x) * next.y -
                        static_cast<long long>(next.x) * current.y;
    }
    return doubled_area;
}

}  // namespace

std::vector<GridCorner> trace_outer_ring(const ObjectGrid& grid,
                                         const std::vector<std::uint32_t>& labels,
                                         std::uint32_t piece) {
    std::vector<GridCorner> outer_ring;
    long long outer_doubled_area = 0;
    // holes run clockwise, so their area is negative and they never win; the largest ring guards
    // pieces that are not 4-connected, and a tie keeps the ring found first
    const std::vector<BoundaryEdge> edges = find_boundary_edges(grid, labels, piece);
    for (const std::vector<BoundaryEdge>& ring : chain_rings(grid, edges)) {
        std::vector<GridCorner> corners = keep_corners_only(ring);
        const long long doubled_area = doubled_signed_area(corners);
        if (doubled_area <= outer_doubled_area) continue;

        outer_doubled_area = doubled_area;
        outer_ring = std::move(corners);
    }
    return outer_ring;
}

}  // namespace vcortex
