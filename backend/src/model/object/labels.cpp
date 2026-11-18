#include "vcortex/model/object/labels.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include "vcortex/geometry/grids.hpp"

namespace vcortex {

std::vector<std::uint32_t> label_objects(const ObjectGrid& grid,
                                         const std::vector<std::uint8_t>& marked_cells) {
    std::vector<std::uint32_t> labels(marked_cells.size(), 0);
    std::uint32_t object_id = 0;
    for (std::size_t index = 0; index < marked_cells.size(); ++index) {
        if (!marked_cells[index] || labels[index]) continue;
        labels[index] = ++object_id;
        std::queue<std::size_t> to_visit;
        to_visit.push(index);
        while (!to_visit.empty()) {
            const std::size_t cell_index = to_visit.front();
            to_visit.pop();
            for (const std::size_t neighbour_index : get_neighbour_indices(grid, cell_index)) {
                if (!marked_cells[neighbour_index] || labels[neighbour_index]) continue;
                labels[neighbour_index] = object_id;
                to_visit.push(neighbour_index);
            }
        }
    }
    return labels;
}

std::vector<std::uint32_t> assign_non_labelled_objects(
    const ObjectGrid& grid, const std::vector<std::uint32_t>& labels,
    const std::vector<std::uint32_t>& region_of_cell) {
    std::vector<std::uint32_t> filled_labels = labels;
    if (labels.empty()) return filled_labels;
    std::uint32_t max_object_id = *std::max_element(labels.begin(), labels.end());

    // every label spreads at once, so a cell joins the label fewest steps away
    std::queue<std::size_t> to_visit;
    for (std::size_t index = 0; index < labels.size(); ++index) {
        if (labels[index]) to_visit.push(index);
    }

    std::size_t next_leftover = 0;
    while (true) {
        if (to_visit.empty()) {
            while (next_leftover < labels.size() &&
                   (!region_of_cell[next_leftover] || filled_labels[next_leftover])) {
                ++next_leftover;
            }
            if (next_leftover == labels.size()) break;
            filled_labels[next_leftover] = ++max_object_id;
            to_visit.push(next_leftover);
        }
        const std::size_t cell_index = to_visit.front();
        to_visit.pop();
        for (const std::size_t neighbour_index : get_neighbour_indices(grid, cell_index)) {
            if (filled_labels[neighbour_index] || !region_of_cell[neighbour_index] ||
                region_of_cell[neighbour_index] != region_of_cell[cell_index]) {
                continue;
            }
            filled_labels[neighbour_index] = filled_labels[cell_index];
            to_visit.push(neighbour_index);
        }
    }
    return filled_labels;
}

namespace {

/// Struct to hold the regions of a height map, where each region is a connected group of cells
/// whose tops step by at most height_step.
struct HeightRegions {
    std::vector<std::uint32_t> region_of_cell;  // cell to region mapper, 0 means no region
    std::vector<std::vector<std::size_t>> cells_of_region;  // index 0 is the unused no-region slot
    std::vector<double> top_sum_of_region;

    double mean_top(std::uint32_t region_id) const {
        return top_sum_of_region[region_id] /
               static_cast<double>(cells_of_region[region_id].size());
    }
};

bool is_top_jump_too_big(const ObjectGrid& grid, std::size_t cell_index,
                         std::size_t neighbour_index, double height_step) {
    return std::abs(grid.top_metres[neighbour_index] - grid.top_metres[cell_index]) > height_step;
}

/// Floods a new region from the seed through cells of the same piece whose tops step by at
/// most height_step, recording its cells and top sum.
void grow_region(const ObjectGrid& grid, const std::vector<std::uint32_t>& labels,
                 double height_step, std::size_t seed_index, HeightRegions& regions) {
    const auto region_id = static_cast<std::uint32_t>(regions.cells_of_region.size());
    regions.cells_of_region.emplace_back();
    regions.top_sum_of_region.push_back(0.0);
    regions.region_of_cell[seed_index] = region_id;

    std::queue<std::size_t> cells_to_visit;
    cells_to_visit.push(seed_index);

    while (!cells_to_visit.empty()) {
        const std::size_t cell_index = cells_to_visit.front();
        cells_to_visit.pop();
        regions.cells_of_region[region_id].push_back(cell_index);
        regions.top_sum_of_region[region_id] += grid.top_metres[cell_index];

        for (const std::size_t neighbour_index : get_neighbour_indices(grid, cell_index)) {
            const bool is_free_with_points =
                !regions.region_of_cell[neighbour_index] && grid.point_count[neighbour_index];
            const bool is_same_piece = labels[neighbour_index] == labels[cell_index];

            if (!is_free_with_points || !is_same_piece ||
                is_top_jump_too_big(grid, cell_index, neighbour_index, height_step)) {
                continue;
            }

            regions.region_of_cell[neighbour_index] = region_id;
            cells_to_visit.push(neighbour_index);
        }
    }
}

/// Splits every piece into regions of smooth tops. Only cells with points have a top; the
/// cells closing filled in stay at region 0.
HeightRegions grow_height_regions(const ObjectGrid& grid, const std::vector<std::uint32_t>& labels,
                                  double height_step) {
    HeightRegions regions;
    regions.region_of_cell.assign(labels.size(), 0);
    regions.cells_of_region.resize(1);
    regions.top_sum_of_region.assign(1, 0.0);

    for (std::size_t index = 0; index < labels.size(); ++index) {
        const bool is_new_seed =
            labels[index] && grid.point_count[index] && !regions.region_of_cell[index];

        if (is_new_seed) grow_region(grid, labels, height_step, index, regions);
    }

    return regions;
}

/// Returns the neighbour region of the same piece whose mean top is closest to this region's,
/// or 0 when the region touches no other region of its piece.
std::uint32_t find_closest_neighbour_region(const ObjectGrid& grid,
                                            const std::vector<std::uint32_t>& labels,
                                            const HeightRegions& regions, std::uint32_t region_id) {
    const double region_mean_top = regions.mean_top(region_id);
    std::uint32_t closest_region_id = 0;
    double closest_top_difference = std::numeric_limits<double>::infinity();

    for (const std::size_t cell_index : regions.cells_of_region[region_id]) {
        for (const std::size_t neighbour_index : get_neighbour_indices(grid, cell_index)) {
            const std::uint32_t neighbour_region_id = regions.region_of_cell[neighbour_index];
            const bool is_other_region_of_same_object =
                neighbour_region_id && neighbour_region_id != region_id &&
                labels[neighbour_index] == labels[cell_index];

            if (!is_other_region_of_same_object) continue;

            const double top_difference =
                std::abs(regions.mean_top(neighbour_region_id) - region_mean_top);

            if (top_difference < closest_top_difference) {
                closest_top_difference = top_difference;
                closest_region_id = neighbour_region_id;
            }
        }
    }

    return closest_region_id;
}

/// Moves every cell and the top sum of one region into another, leaving the first empty.
void move_region_into(HeightRegions& regions, std::uint32_t from_region_id,
                      std::uint32_t into_region_id) {
    std::vector<std::size_t>& from_cells = regions.cells_of_region[from_region_id];
    std::vector<std::size_t>& into_cells = regions.cells_of_region[into_region_id];

    for (const std::size_t cell_index : from_cells) {
        regions.region_of_cell[cell_index] = into_region_id;
    }

    into_cells.insert(into_cells.end(), from_cells.begin(), from_cells.end());
    regions.top_sum_of_region[into_region_id] += regions.top_sum_of_region[from_region_id];
    from_cells.clear();
    regions.top_sum_of_region[from_region_id] = 0.0;
}

/// Merges every region smaller than min_piece_cells into its closest neighbour region.
/// cell tops are noisy, so a small region is mostly noise
void merge_small_regions(const ObjectGrid& grid, const std::vector<std::uint32_t>& labels,
                         int min_piece_cells, HeightRegions& regions) {
    const auto min_region_cells = static_cast<std::size_t>(min_piece_cells);

    for (std::uint32_t region_id = 1; region_id < regions.cells_of_region.size(); ++region_id) {
        const std::size_t region_cell_count = regions.cells_of_region[region_id].size();

        // an empty region was already merged into another
        if (region_cell_count == 0 || region_cell_count >= min_region_cells) continue;

        const std::uint32_t closest_region_id =
            find_closest_neighbour_region(grid, labels, regions, region_id);

        // no neighbour region: the whole piece is this small region, so it stays as it is
        if (!closest_region_id) continue;

        move_region_into(regions, region_id, closest_region_id);
    }
}

}  // namespace

std::vector<std::uint32_t> split_by_height(const ObjectGrid& grid,
                                           const std::vector<std::uint32_t>& labels,
                                           double height_step, int min_piece_cells) {
    HeightRegions regions = grow_height_regions(grid, labels, height_step);
    merge_small_regions(grid, labels, min_piece_cells, regions);

    // the cells closing filled in have no top, so they join the nearest region
    return assign_non_labelled_objects(grid, regions.region_of_cell, labels);
}

}  // namespace vcortex
