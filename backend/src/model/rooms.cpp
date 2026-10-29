#include "vcortex/model/rooms.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <string>

namespace vcortex {

namespace {

constexpr double kEps = 1e-9;

// walls leave split_walls_objects parallel to x or y, so each is a box
struct Body {
    bool vertical = false;  // runs along y, constant x
    double centre = 0.0, half = 0.0;
    double from = 0.0, to = 0.0;
};

Body body_of(const Wall& w) {
    Body b;
    b.vertical = std::abs(w.start.x - w.end.x) < kEps;
    b.centre = b.vertical ? w.start.x : w.start.y;
    b.half = w.thickness / 2.0;
    b.from = b.vertical ? std::min(w.start.y, w.end.y) : std::min(w.start.x, w.end.x);
    b.to = b.vertical ? std::max(w.start.y, w.end.y) : std::max(w.start.x, w.end.x);
    return b;
}

// 2d occupation matrix
struct Mask {
    const Grid& grid;
    std::vector<int> label;  // -1 blocked, 0 free and unseen, k room k

    explicit Mask(const Grid& g)
        : grid(g), label(static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), 0) {}

    int& at(int ix, int iy) {
        return label[static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid.width) +
                     static_cast<std::size_t>(ix)];
    }

    // every cell the box touches, not only cells whose centre it covers: a 7 cm wall can
    // fall between two centres and leave a leak
    void block(double x0, double x1, double y0, double y1) {
        const auto first = [&](double v, double o) {
            return static_cast<int>(std::floor((v - o) / grid.cell + kEps));
        };
        const auto last = [&](double v, double o) {
            return static_cast<int>(std::ceil((v - o) / grid.cell - kEps)) - 1;
        };
        const int ix0 = std::max(0, first(x0, grid.origin.x));
        const int ix1 = std::min(grid.width - 1, last(x1, grid.origin.x));
        const int iy0 = std::max(0, first(y0, grid.origin.y));
        const int iy1 = std::min(grid.height - 1, last(y1, grid.origin.y));
        for (int iy = iy0; iy <= iy1; ++iy) {
            for (int ix = ix0; ix <= ix1; ++ix) at(ix, iy) = -1;
        }
    }

    void block(const Body& b) {
        if (b.vertical) {
            block(b.centre - b.half, b.centre + b.half, b.from, b.to);
        } else {
            block(b.from, b.to, b.centre - b.half, b.centre + b.half);
        }
    }
};

// two pieces of one wall line with a short gap between them: a door, closed so the fill stops
void close_doors(const std::vector<Body>& bodies, double door_max, Mask& m) {
    for (const Body& a : bodies) {
        for (const Body& b : bodies) {
            const double gap = b.from - a.to;
            if (&a == &b || a.vertical != b.vertical || gap <= 0.0 || gap > door_max + kEps ||
                std::abs(a.centre - b.centre) > a.half + b.half) {
                continue;
            }
            Body door;
            door.vertical = a.vertical;
            door.centre = (a.centre + b.centre) / 2.0;
            door.half = std::min(a.half, b.half);
            door.from = a.to;
            door.to = b.from;
            m.block(door);
        }
    }
}

struct Fill {
    std::size_t cells = 0;
    int ix0 = 0, ix1 = 0, iy0 = 0, iy1 = 0;
    bool leaked = false;
};

// 4 neighbours, not 8: diagonal steps slip between two wall cells that meet at a corner
Fill flood(Mask& m, int sx, int sy, int k) {
    const Grid& g = m.grid;
    Fill f{0, sx, sx, sy, sy, false};
    std::queue<std::pair<int, int>> todo;
    // labelled when queued, not when taken out, so no cell is queued twice
    m.at(sx, sy) = k;
    todo.push({sx, sy});
    while (!todo.empty()) {
        const auto [ix, iy] = todo.front();
        todo.pop();
        ++f.cells;
        f.ix0 = std::min(f.ix0, ix);
        f.ix1 = std::max(f.ix1, ix);
        f.iy0 = std::min(f.iy0, iy);
        f.iy1 = std::max(f.iy1, iy);
        if (ix == 0 || iy == 0 || ix == g.width - 1 || iy == g.height - 1) f.leaked = true;
        const int step[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& s : step) {
            const int nx = ix + s[0], ny = iy + s[1];
            if (nx < 0 || ny < 0 || nx >= g.width || ny >= g.height || m.at(nx, ny) != 0) continue;
            m.at(nx, ny) = k;
            todo.push({nx, ny});
        }
    }
    return f;
}

// the face on the room side lies within wall_touch_distance of the outline, and the wall
// runs along that side
bool touches(const Body& b, double x0, double x1, double y0, double y1, double d) {
    if (b.vertical) {
        if (std::min(b.to, y1) - std::max(b.from, y0) <= 0.0) return false;
        return std::abs(b.centre + b.half - x0) <= d + kEps ||
               std::abs(b.centre - b.half - x1) <= d + kEps;
    }
    if (std::min(b.to, x1) - std::max(b.from, x0) <= 0.0) return false;
    return std::abs(b.centre + b.half - y0) <= d + kEps ||
           std::abs(b.centre - b.half - y1) <= d + kEps;
}

}  // namespace

std::vector<Room> find_rooms(const std::vector<Wall>& walls, const Grid& grid,
                             const Trajectory& aligned_traj, double door_max,
                             const RoomParams& p) {
    std::vector<Body> bodies;
    bodies.reserve(walls.size());
    for (const Wall& w : walls) bodies.push_back(body_of(w));

    Mask m(grid);
    for (const Body& b : bodies) m.block(b);
    close_doors(bodies, door_max, m);

    std::vector<Room> out;
    int k = 0;
    for (const Pose& pose : aligned_traj) {
        const int ix = static_cast<int>(std::floor((pose.x - grid.origin.x) / grid.cell));
        const int iy = static_cast<int>(std::floor((pose.y - grid.origin.y) / grid.cell));
        // a pose close to a wall can fall on its drawn cells; later poses cover the room
        if (ix < 0 || iy < 0 || ix >= grid.width || iy >= grid.height || m.at(ix, iy) != 0) {
            continue;
        }
        const Fill f = flood(m, ix, iy, ++k);
        if (static_cast<double>(f.cells) * grid.cell * grid.cell < p.min_area) continue;

        // TODO: rectilinear outline that follows the cells; the box overstates l-shaped rooms
        const double x0 = grid.origin.x + f.ix0 * grid.cell;
        const double x1 = grid.origin.x + (f.ix1 + 1) * grid.cell;
        const double y0 = grid.origin.y + f.iy0 * grid.cell;
        const double y1 = grid.origin.y + (f.iy1 + 1) * grid.cell;
        Room r;
        r.id = "room-" + std::to_string(out.size() + 1);
        r.outline = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
        r.closed = !f.leaked;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            if (touches(bodies[i], x0, x1, y0, y1, p.wall_touch_distance)) {
                r.wall_ids.push_back(walls[i].id);
            }
        }
        out.push_back(r);
    }
    return out;
}

}  // namespace vcortex
