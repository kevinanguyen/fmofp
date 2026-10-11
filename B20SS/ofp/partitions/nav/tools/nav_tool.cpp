// fmofp_nav_tool: the navigation partition's pure functions on stdin/stdout,
// so the Python tests can run the original Python beside them
// (B20SS/fmofp_bus/tests/test_nav_oracle.py). Test support, not flight code.
//
// One command per line; every number is printed with 17 significant digits.
//   sat <id>                              -> x y z clock_bias_s
//   elev <rx> <ry> <rz> <sx> <sy> <sz>    -> elevation_deg
//   wgs84 <x> <y> <z>                     -> lat lon alt_ft
//   ecef <lat> <lon> <alt_ft>             -> x y z
//   dr <gs_kt> <heading> <vs_fpm> <lat>   -> lat_dps lon_dps alt_fps
//   solve4 <a00> .. <a33> <b0> .. <b3>    -> x0 x1 x2 x3            (or "error <status>")
//   wls <x> <y> <z> <cb_s> <n> <id> <pr> ... -> x y z cb_s iterations converged  (or "error <status>")
//   bancroft <n> <id> <pr> ...             -> x y z cb_s                (or "error <status>")
//   fusion                                 resets the fusion filter
//   predict <dt> <lat_dps> <lon_dps> <alt_fps>
//   update <lat> <lon> <alt_ft>
//   get                                   -> lat lon alt_ft fix_count

#include <cstdio>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <fmofp/bus/status.hpp>
#include <fmofp/nav/gps_solver.hpp>
#include <fmofp/nav/navigation.hpp>

namespace {

using fmofp::Status;
namespace nav = fmofp::nav;

void print(std::initializer_list<double> values) {
    bool first = true;
    for (const double v : values) {
        std::printf(first ? "%.17g" : " %.17g", v);
        first = false;
    }
    std::printf("\n");
}

bool read(std::istringstream& in, std::vector<double>& out, std::size_t count) {
    out.clear();
    double value = 0.0;
    while (out.size() < count && in >> value) {
        out.push_back(value);
    }
    return out.size() == count;
}

}  // namespace

int main() {
    const nav::Constellation sats = nav::build_constellation();
    nav::NavFusion fusion;
    fusion.init();
    std::string line;
    std::vector<double> a;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd) || cmd.empty() || cmd[0] == '#') {
            continue;
        }
        if (cmd == "sat" && read(in, a, 1)) {
            const auto id = static_cast<std::size_t>(a[0]);
            if (id < 1 || id > nav::kSatellites) {
                std::printf("error kInvalidArgument\n");
                continue;
            }
            const nav::Satellite& s = sats[id - 1];
            print({s.position.x, s.position.y, s.position.z, s.clock_bias_s});
        } else if (cmd == "elev" && read(in, a, 6)) {
            print({nav::elevation_deg(nav::Ecef{a[0], a[1], a[2]}, nav::Ecef{a[3], a[4], a[5]})});
        } else if (cmd == "wgs84" && read(in, a, 3)) {
            const nav::Geodetic g = nav::ecef_to_wgs84(nav::Ecef{a[0], a[1], a[2]});
            print({g.latitude_deg, g.longitude_deg, g.altitude_ft});
        } else if (cmd == "ecef" && read(in, a, 3)) {
            nav::Geodetic g;
            g.latitude_deg = a[0];
            g.longitude_deg = a[1];
            g.altitude_ft = a[2];
            const nav::Ecef p = nav::wgs84_to_ecef(g);
            print({p.x, p.y, p.z});
        } else if (cmd == "dr" && read(in, a, 4)) {
            const nav::DeadReckoningRates r = nav::dead_reckoning_rates(a[0], a[1], a[2], a[3]);
            print({r.lat_dps, r.lon_dps, r.alt_fps});
        } else if (cmd == "solve4" && read(in, a, 20)) {
            nav::Matrix4 m{};
            nav::Vector4 b{};
            nav::Vector4 x{};
            for (std::size_t r = 0; r < 4; ++r) {
                for (std::size_t c = 0; c < 4; ++c) {
                    m[r][c] = a[r * 4 + c];
                }
                b[r] = a[16 + r];
            }
            const Status s = nav::solve_4x4(m, b, x);
            if (s != Status::kOk) {
                std::printf("error %s\n", fmofp::to_string(s));
            } else {
                print({x[0], x[1], x[2], x[3]});
            }
        } else if (cmd == "wls" && read(in, a, 5)) {
            const auto n = static_cast<std::size_t>(a[4]);
            std::vector<double> pairs;
            if (n > nav::kSatellites || !read(in, pairs, 2 * n)) {
                std::printf("error kInvalidArgument\n");
                continue;
            }
            std::vector<nav::Observation> obs(n);
            for (std::size_t i = 0; i < n; ++i) {
                obs[i].satellite = static_cast<std::uint8_t>(pairs[2 * i]);
                obs[i].pseudorange_m = pairs[2 * i + 1];
            }
            nav::Fix fix;
            const Status s = nav::wls_solve(sats, obs.data(), n, nav::Ecef{a[0], a[1], a[2]}, a[3], fix);
            if (s != Status::kOk) {
                std::printf("error %s\n", fmofp::to_string(s));
            } else {
                print({fix.position.x, fix.position.y, fix.position.z, fix.clock_bias_s,
                       static_cast<double>(fix.iterations), fix.converged ? 1.0 : 0.0});
            }
        } else if (cmd == "bancroft" && read(in, a, 1)) {
            const auto n = static_cast<std::size_t>(a[0]);
            std::vector<double> pairs;
            if (n > nav::kSatellites || !read(in, pairs, 2 * n)) {
                std::printf("error kInvalidArgument\n");
                continue;
            }
            std::vector<nav::Observation> obs(n);
            for (std::size_t i = 0; i < n; ++i) {
                obs[i].satellite = static_cast<std::uint8_t>(pairs[2 * i]);
                obs[i].pseudorange_m = pairs[2 * i + 1];
            }
            nav::Fix fix;
            const Status s = nav::bancroft(sats, obs.data(), n, fix);
            if (s != Status::kOk) {
                std::printf("error %s\n", fmofp::to_string(s));
            } else {
                print({fix.position.x, fix.position.y, fix.position.z, fix.clock_bias_s});
            }
        } else if (cmd == "fusion") {
            fusion.init();
        } else if (cmd == "predict" && read(in, a, 4)) {
            fusion.predict(a[0], a[1], a[2], a[3]);
        } else if (cmd == "update" && read(in, a, 3)) {
            fusion.update_gps(a[0], a[1], a[2]);
        } else if (cmd == "get") {
            print({fusion.latitude(), fusion.longitude(), fusion.altitude_ft(), static_cast<double>(fusion.fix_count())});
        } else {
            std::printf("error unknown command\n");
        }
        std::fflush(stdout);
    }
    return 0;
}
