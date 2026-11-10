#include "vcortex/model/objects.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl_bridge.hpp"
#include "vcortex/geometry/points.hpp"
#include "vcortex/model/candidates.hpp"
#include "vcortex/model/object_grid.hpp"

namespace vcortex {

namespace {

constexpr double kEps = 1e-9;

bool overlaps(double a0, double a1, double b0, double b1) { return a0 < b1 && b0 < a1; }

// a wall in the aligned frame as a line of constant x or y, so a box side can be measured
// against its body; walls are axis-aligned here, they turn only in the map frame
struct WallLine {
    Axis axis = Axis::X;
    double position = 0.0;        // the constant coordinate
    double from = 0.0, to = 0.0;  // extent along the wall
    double half_thickness = 0.0;
};

WallLine wall_line(const Wall& wall) {
    WallLine line;
    line.axis = wall.start.x == wall.end.x ? Axis::X : Axis::Y;
    line.position = line.axis == Axis::X ? wall.start.x : wall.start.y;
    const double along_start = line.axis == Axis::X ? wall.start.y : wall.start.x;
    const double along_end = line.axis == Axis::X ? wall.end.y : wall.end.x;
    line.from = std::min(along_start, along_end);
    line.to = std::max(along_start, along_end);
    line.half_thickness = wall.thickness / 2.0;
    return line;
}

// gap between the box range [box_from, box_to] across the wall and the wall body
double box_to_wall_distance(double box_from, double box_to, const WallLine& line) {
    const double body_from = line.position - line.half_thickness;
    const double body_to = line.position + line.half_thickness;
    return std::max({0.0, body_from - box_to, box_from - body_to});
}

}  // namespace

std::vector<Object> find_objects(const std::vector<Point>& furniture_points, const Plane& floor,
                                 const std::vector<Wall>& walls, const ObjectParams& parameters) {
    std::vector<Object> objects;
    if (furniture_points.empty()) return objects;

    // the margin covers the widest neighbourhood closing and opening read, plus one empty cell
    const int close_radius_cells =
        static_cast<int>(std::lround(parameters.close_radius / parameters.cell));
    const int split_radius_cells =
        static_cast<int>(std::lround(parameters.split_radius / parameters.cell));
    const int padding_cells = std::max(close_radius_cells, split_radius_cells) + 1;
    // TODO: cut pieces on this grid and drop the point clustering below
    const ObjectGrid grid = build_object_grid(furniture_points, floor, parameters.cell,
                                              padding_cells, parameters.cell_min_points);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = to_pcl(furniture_points);
    const pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setClusterTolerance(parameters.cluster_tolerance);
    cluster_extraction.setMinClusterSize(static_cast<pcl::uindex_t>(parameters.cluster_min_points));
    cluster_extraction.setMaxClusterSize(std::numeric_limits<pcl::uindex_t>::max());
    cluster_extraction.setSearchMethod(tree);
    cluster_extraction.setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusters;
    cluster_extraction.extract(clusters);

    std::vector<WallLine> wall_lines;
    wall_lines.reserve(walls.size());
    for (const Wall& wall : walls) wall_lines.push_back(wall_line(wall));

    objects.reserve(clusters.size());
    for (const pcl::PointIndices& cluster : clusters) {
        const std::vector<Point> points = pick(furniture_points, cluster.indices);

        const Bounds bounds = point_bounds(points);

        Object object;
        object.center = {(bounds.min_x + bounds.max_x) / 2.0, (bounds.min_y + bounds.max_y) / 2.0};
        object.size = {static_cast<double>(bounds.max_x - bounds.min_x), static_cast<double>(bounds.max_y - bounds.min_y)};
        object.bottom_z = bounds.min_z;
        object.top_z_seen = bounds.max_z;
        object.points = points.size();

        double best_distance = parameters.against_wall_distance + kEps;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            const WallLine& line = wall_lines[i];
            const bool is_along = line.axis == Axis::X
                                      ? overlaps(bounds.min_y, bounds.max_y, line.from, line.to)
                                      : overlaps(bounds.min_x, bounds.max_x, line.from, line.to);
            const double distance = line.axis == Axis::X
                                        ? box_to_wall_distance(bounds.min_x, bounds.max_x, line)
                                        : box_to_wall_distance(bounds.min_y, bounds.max_y, line);
            if (is_along && distance <= best_distance) {
                best_distance = distance;
                object.against_wall_id = walls[i].id;
            }
        }
        objects.push_back(object);
    }

    std::sort(objects.begin(), objects.end(), [](const Object& a, const Object& b) {
        return std::tie(a.center.x, a.center.y) < std::tie(b.center.x, b.center.y);
    });
    for (std::size_t i = 0; i < objects.size(); ++i) {
        objects[i].id = "object-" + std::to_string(i + 1);
    }
    return objects;
}

}  // namespace vcortex
