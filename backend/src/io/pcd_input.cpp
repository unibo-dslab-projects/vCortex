#include "vcortex/io/pcd_input.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

namespace vcortex {

namespace {

// Trajectory promises one pose per 0.2 m of travel (snapshot.hpp)
constexpr double kPoseSpacing = 0.2;

// by name and stored type: map_builder writes hits as an integer, point-lio's files carry
// fields we skip (normals, curvature)
class FieldReader {
public:
    FieldReader(const pcl::PCLPointCloud2& cloud, const std::string& name) : cloud_(cloud) {
        for (const pcl::PCLPointField& f : cloud.fields) {
            if (f.name == name) field_ = &f;
        }
    }
    bool present() const { return field_ != nullptr; }

    double at(std::size_t i) const {
        const std::uint8_t* p = cloud_.data.data() + i * cloud_.point_step + field_->offset;
        switch (field_->datatype) {
            case pcl::PCLPointField::INT8: return read<std::int8_t>(p);
            case pcl::PCLPointField::UINT8: return read<std::uint8_t>(p);
            case pcl::PCLPointField::INT16: return read<std::int16_t>(p);
            case pcl::PCLPointField::UINT16: return read<std::uint16_t>(p);
            case pcl::PCLPointField::INT32: return read<std::int32_t>(p);
            case pcl::PCLPointField::UINT32: return read<std::uint32_t>(p);
            case pcl::PCLPointField::FLOAT32: return read<float>(p);
            case pcl::PCLPointField::FLOAT64: return read<double>(p);
            default:
                throw std::runtime_error("load_offline: field '" + field_->name +
                                         "' has an unsupported type");
        }
    }

private:
    // memcpy: point_step need not keep fields aligned
    template <typename T>
    static double read(const std::uint8_t* p) {
        T v;
        std::memcpy(&v, p, sizeof v);
        return static_cast<double>(v);
    }

    const pcl::PCLPointCloud2& cloud_;
    const pcl::PCLPointField* field_ = nullptr;
};

Snapshot read_pcd(const std::filesystem::path& file) {
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(file.string(), cloud) != 0) {
        throw std::runtime_error("load_offline: cannot read " + file.string());
    }
    const FieldReader x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    if (!x.present() || !y.present() || !z.present()) {
        throw std::runtime_error("load_offline: " + file.string() + " lacks x, y or z");
    }
    const FieldReader hits(cloud, "hits"), intensity(cloud, "intensity");

    Snapshot snap;
    snap.has_hits = hits.present();
    snap.has_intensity = intensity.present();
    const std::size_t n = static_cast<std::size_t>(cloud.width) * cloud.height;
    snap.points.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        Point p;
        p.x = static_cast<float>(x.at(i));
        p.y = static_cast<float>(y.at(i));
        p.z = static_cast<float>(z.at(i));
        // organised clouds mark missing returns with nan
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (snap.has_intensity) p.intensity = static_cast<float>(intensity.at(i));
        if (snap.has_hits) {
            p.hits = static_cast<std::uint16_t>(
                std::min<double>(hits.at(i), std::numeric_limits<std::uint16_t>::max()));
        }
        snap.points.push_back(p);
    }
    return snap;
}

// "1792420523.201106071" as text: through a double the nanoseconds would be lost
std::int64_t stamp_ns(const std::string& s) {
    const std::size_t dot = s.find('.');
    const std::int64_t sec = std::stoll(s.substr(0, dot));
    std::int64_t ns = 0;
    if (dot != std::string::npos) {
        std::string frac = s.substr(dot + 1, 9);
        frac.append(9 - frac.size(), '0');
        ns = std::stoll(frac);
    }
    return sec * 1'000'000'000 + ns;
}

// tum: "stamp x y z qx qy qz qw" per line; yaw only, point-lio has already levelled the map
Trajectory read_tum(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("load_offline: cannot read " + file.string());
    Trajectory traj;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ls(line);
        std::string stamp;
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
        Pose p;
        if (!(ls >> stamp >> p.x >> p.y >> p.z >> qx >> qy >> qz >> qw)) {
            throw std::runtime_error("load_offline: " + file.string() + " line " +
                                     std::to_string(line_no) + " is not a tum pose");
        }
        if (!traj.empty() && std::hypot(p.x - traj.back().x, p.y - traj.back().y) < kPoseSpacing) {
            continue;
        }
        p.time_ns = stamp_ns(stamp);
        p.yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        traj.push_back(p);
    }
    return traj;
}

}  // namespace

OfflineInput load_offline(const std::filesystem::path& pcd_file) {
    OfflineInput out;
    out.snapshot = read_pcd(pcd_file);
    // optional: without it the model falls back to the outermost-line rule
    const std::filesystem::path tum = pcd_file.parent_path() / "trajectory.tum";
    if (std::filesystem::exists(tum)) out.trajectory = read_tum(tum);
    return out;
}

}  // namespace vcortex
