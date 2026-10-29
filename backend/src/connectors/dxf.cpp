#include "vcortex/connectors/dxf.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

namespace vcortex {

namespace {

constexpr const char* kWalls = "WALLS";
constexpr const char* kRooms = "ROOMS";
constexpr const char* kObjects = "OBJECTS";
constexpr double kTextHeight = 0.25;  // metres

// ascii dxf is a list of pairs: group code on one line, value on the next
void pair(std::string& out, int code, const std::string& value) {
    out += std::to_string(code);
    out += '\n';
    out += value;
    out += '\n';
}

// millimetres like the twin json; snprintf uses the "C" locale, so the separator is a dot
void pair(std::string& out, int code, double value) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.3f", std::round(value * 1000.0) / 1000.0 + 0.0);
    pair(out, code, std::string(buf));
}

// a line break inside a value would shift every following pair
std::string one_line(std::string s) {
    for (char& c : s) {
        if (c == '\n' || c == '\r') c = ' ';
    }
    return s;
}

// r2000 readers (ezdxf, cad programs) refuse entities without their 100 subclass markers
void line(std::string& out, const char* layer, Vec2 a, Vec2 b) {
    pair(out, 0, "LINE");
    pair(out, 100, "AcDbEntity");
    pair(out, 8, layer);
    pair(out, 100, "AcDbLine");
    pair(out, 10, a.x);
    pair(out, 20, a.y);
    pair(out, 30, 0.0);
    pair(out, 11, b.x);
    pair(out, 21, b.y);
    pair(out, 31, 0.0);
}

void polyline(std::string& out, const char* layer, const std::vector<Vec2>& vertices) {
    pair(out, 0, "LWPOLYLINE");
    pair(out, 100, "AcDbEntity");
    pair(out, 8, layer);
    pair(out, 100, "AcDbPolyline");
    pair(out, 90, std::to_string(vertices.size()));
    pair(out, 70, "1");  // closed: the last vertex joins the first
    for (const Vec2& v : vertices) {
        pair(out, 10, v.x);
        pair(out, 20, v.y);
    }
}

// 72 = 1 and 73 = 2 centre the text on 11/21; 10/20 must still be there
void text(std::string& out, const char* layer, Vec2 at, const std::string& s) {
    pair(out, 0, "TEXT");
    pair(out, 100, "AcDbEntity");
    pair(out, 8, layer);
    pair(out, 100, "AcDbText");
    pair(out, 10, at.x);
    pair(out, 20, at.y);
    pair(out, 30, 0.0);
    pair(out, 40, kTextHeight);
    pair(out, 1, one_line(s));
    pair(out, 72, "1");
    pair(out, 11, at.x);
    pair(out, 21, at.y);
    pair(out, 31, 0.0);
    pair(out, 100, "AcDbText");  // TEXT repeats its subclass marker before 73
    pair(out, 73, "2");
}

// both faces plus the two ends, as an architect draws a wall, instead of the centreline alone
void wall(std::string& out, const Wall& w) {
    const double dx = w.end.x - w.start.x, dy = w.end.y - w.start.y;
    const double len = std::hypot(dx, dy);
    if (!(len > 0.0)) return;  // no direction to offset the faces along
    const double h = w.thickness / 2.0;
    const double nx = -dy / len * h, ny = dx / len * h;
    const Vec2 s1{w.start.x + nx, w.start.y + ny}, s2{w.start.x - nx, w.start.y - ny};
    const Vec2 e1{w.end.x + nx, w.end.y + ny}, e2{w.end.x - nx, w.end.y - ny};
    line(out, kWalls, s1, e1);
    line(out, kWalls, s2, e2);
    line(out, kWalls, s1, s2);
    line(out, kWalls, e1, e2);
}

// four LINEs, not a closed polyline: importers that ignore layers read every closed polyline as
// a room
void object(std::string& out, const Object& o) {
    const double c = std::cos(o.yaw), s = std::sin(o.yaw);
    const double hx = o.size.x / 2.0, hy = o.size.y / 2.0;
    const double sx[4] = {-1, 1, 1, -1}, sy[4] = {-1, -1, 1, 1};
    std::array<Vec2, 4> k;
    for (std::size_t i = 0; i < 4; ++i) {
        const double lx = sx[i] * hx, ly = sy[i] * hy;
        k[i] = {o.center.x + c * lx - s * ly, o.center.y + s * lx + c * ly};
    }
    for (std::size_t i = 0; i < 4; ++i) line(out, kObjects, k[i], k[(i + 1) % 4]);
}

void room(std::string& out, const Room& r) {
    if (r.outline.size() < 3) return;
    polyline(out, kRooms, r.outline);
    Vec2 mid;
    for (const Vec2& v : r.outline) {
        mid.x += v.x;
        mid.y += v.y;
    }
    mid.x /= static_cast<double>(r.outline.size());
    mid.y /= static_cast<double>(r.outline.size());
    text(out, kRooms, mid, r.name.value_or(r.id));
}

}  // namespace

// TODO: handles and the TABLES, BLOCKS and OBJECTS sections; ezdxf and the open cad programs
// read the file without them, AutoCAD may not
std::string to_dxf(const ModelResult& result) {
    std::string out;
    pair(out, 0, "SECTION");
    pair(out, 2, "HEADER");
    pair(out, 9, "$ACADVER");
    pair(out, 1, "AC1015");  // r2000, the first version with LWPOLYLINE
    pair(out, 9, "$INSUNITS");
    pair(out, 70, "6");  // metres
    pair(out, 0, "ENDSEC");

    pair(out, 0, "SECTION");
    pair(out, 2, "ENTITIES");
    for (const Level& level : result.building.levels) {
        for (const Wall& w : level.walls) wall(out, w);
        for (const Room& r : level.rooms) room(out, r);
        for (const Object& o : level.objects) object(out, o);
    }
    pair(out, 0, "ENDSEC");
    pair(out, 0, "EOF");
    return out;
}

}  // namespace vcortex
