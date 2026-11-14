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

std::vector<std::uint32_t> assign_non_labelled_objects(const ObjectGrid& grid,
                                                       const std::vector<std::uint32_t>& labels,
                                                       const std::vector<std::uint8_t>& occupied) {
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
                   (!occupied[next_leftover] || filled_labels[next_leftover])) {
                ++next_leftover;
            }
            if (next_leftover == labels.size()) break;
            filled_labels[next_leftover] = ++max_object_id;
            to_visit.push(next_leftover);
        }
        const std::size_t cell_index = to_visit.front();
        to_visit.pop();
        for (const std::size_t neighbour_index : get_neighbour_indices(grid, cell_index)) {
            if (filled_labels[neighbour_index] || !occupied[neighbour_index]) continue;
            filled_labels[neighbour_index] = filled_labels[cell_index];
            to_visit.push(neighbour_index);
        }
    }
    return filled_labels;
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
            for (const std::size_t neighbour_index : get_neighbour_indices(grid, cell_index)) {
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
            for (const std::size_t neighbour_index : get_neighbour_indices(grid, cell_index)) {
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

}  // namespace vcortex
