// fmofp_mission_tool: the mission computer's route, targeting and order-of-battle
// logic on stdin/stdout, so the Python tests can run the original handleRoute.py,
// targeting.py and orderOfBattle.py beside them
// (B20SS/fmofp_bus/tests/test_mission_oracle.py). Test support, not flight code.
//
// One command per line; every number is printed with 17 significant digits.
//   route_clear                                   -> ok
//   route_add <name> <lat> <lon> <alt> <type>     -> id            (type: NAV IP TGT EGR ALT REF HOME)
//   route_remove <id>                             -> 0|1
//   route_pos <lat> <lon> <alt> <gs>              -> active distance bearing remaining
//   route_insert_target <name> <lat> <lon> <alt>  -> id index
//   route_deviate <lat> <lon> <radius>            -> adjusted
//   route_dump                                    -> lines "id lat lon alt ete(-1 none)" then "end"
//   tgt_clear                                     -> ok
//   tgt_add <kind> <lat> <lon> <alt> <priority> <frame> -> id    (kind as the enum's number)
//   tgt_acquire <id> <frame>                      -> 0|1
//   tgt_engage <id> <weapon>                      -> 0|1
//   tgt_disengage <id>                            -> 0|1
//   tgt_bda <id>                                  -> 0|1
//   tgt_assess <id> <destroyed>                   -> 0|1
//   tgt_update <id> <lat> <lon> <alt> <speed> <heading> <quality> <frame> -> 0|1
//   tgt_tick <frame>                              -> changed(0|1)
//   tgt_dump                                      -> lines "id status quality" in prioritised order, then "end"
//   oob_clear                                     -> ok
//   oob_add <kind> <affiliation> <lat> <lon> <alt> <threat> -> id
//   oob_threats <min_level>                       -> the ids, then "end"
//   oob_summary                                   -> friendly enemy neutral

#include <algorithm>
#include <array>
#include <cstdio>
#include <initializer_list>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <fmofp/mission/mission_computer.hpp>

namespace {

namespace mission = fmofp::mission;
namespace icd = fmofp::icd;

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

icd::WaypointType waypoint_type(const std::string& text) {
    static const std::array<const char*, 7> names = {"NAV", "IP", "TGT", "EGR", "ALT", "REF", "HOME"};
    const auto it = std::find_if(names.begin(), names.end(), [&text](const char* n) { return text == n; });
    return it == names.end() ? icd::WaypointType::NAV : static_cast<icd::WaypointType>(it - names.begin());
}

std::uint8_t byte(double v) { return static_cast<std::uint8_t>(v); }

// The position of a waypoint in the route, -1 when it is not there.
long index_of(const mission::RouteManager& route, std::uint8_t id) {
    std::vector<std::size_t> indices(route.count());
    std::iota(indices.begin(), indices.end(), std::size_t{0});
    const auto it = std::find_if(indices.begin(), indices.end(),
                                 [&route, id](std::size_t i) { return route.waypoint(i).id == id; });
    return it == indices.end() ? -1L : static_cast<long>(*it);
}

}  // namespace

int main() {
    mission::RouteManager route;
    route.init();
    mission::Targeting targeting;
    targeting.init();
    mission::OrderOfBattle oob;
    oob.init();
    std::string line;
    std::vector<double> a;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd) || cmd.empty() || cmd[0] == '#') {
            continue;
        }
        if (cmd == "route_clear") {
            route.init();
            std::printf("ok\n");
        } else if (cmd == "route_add") {
            std::string name;
            std::string type;
            if (in >> name && read(in, a, 3) && in >> type) {
                print({static_cast<double>(
                    route.add(mission::make_name(name.c_str()), a[0], a[1], a[2], waypoint_type(type), true))});
            } else {
                std::printf("error arguments\n");
            }
        } else if (cmd == "route_remove" && read(in, a, 1)) {
            print({route.remove(byte(a[0])) ? 1.0 : 0.0});
        } else if (cmd == "route_pos" && read(in, a, 4)) {
            route.update_position(a[0], a[1], a[2], a[3]);
            print({static_cast<double>(route.active_index()), route.distance_to_next_nm(), route.bearing_to_next_deg(),
                   static_cast<double>(route.remaining())});
        } else if (cmd == "route_insert_target") {
            std::string name;
            if (in >> name && read(in, a, 3)) {
                const std::uint8_t id = route.insert_target(mission::make_name(name.c_str()), a[0], a[1], a[2]);
                print({static_cast<double>(id), static_cast<double>(index_of(route, id))});
            } else {
                std::printf("error arguments\n");
            }
        } else if (cmd == "route_deviate" && read(in, a, 3)) {
            print({static_cast<double>(route.deviate_from_threat(a[0], a[1], a[2]))});
        } else if (cmd == "route_dump") {
            for (std::size_t i = 0; i < route.count(); ++i) {
                const mission::Waypoint& wp = route.waypoint(i);
                print({static_cast<double>(wp.id), wp.lat, wp.lon, wp.alt_ft, wp.has_ete ? wp.ete_min : -1.0});
            }
            std::printf("end\n");
        } else if (cmd == "tgt_clear") {
            targeting.init();
            std::printf("ok\n");
        } else if (cmd == "tgt_add" && read(in, a, 6)) {
            print({static_cast<double>(targeting.add(static_cast<icd::TargetKind>(byte(a[0])), a[1], a[2], a[3],
                                                     byte(a[4]), static_cast<std::uint32_t>(a[5])))});
        } else if (cmd == "tgt_acquire" && read(in, a, 2)) {
            print({targeting.acquire(byte(a[0]), static_cast<std::uint32_t>(a[1])) ? 1.0 : 0.0});
        } else if (cmd == "tgt_engage" && read(in, a, 2)) {
            print({targeting.engage(byte(a[0]), static_cast<icd::Weapon>(byte(a[1]))) ? 1.0 : 0.0});
        } else if (cmd == "tgt_disengage" && read(in, a, 1)) {
            print({targeting.disengage(byte(a[0])) ? 1.0 : 0.0});
        } else if (cmd == "tgt_bda" && read(in, a, 1)) {
            print({targeting.mark_bda_pending(byte(a[0])) ? 1.0 : 0.0});
        } else if (cmd == "tgt_assess" && read(in, a, 2)) {
            print({targeting.assess_damage(byte(a[0]), a[1] != 0.0) ? 1.0 : 0.0});
        } else if (cmd == "tgt_update" && read(in, a, 8)) {
            print({targeting.update_from_radar(byte(a[0]), a[1], a[2], a[3], a[4], a[5], a[6],
                                               static_cast<std::uint32_t>(a[7]))
                       ? 1.0
                       : 0.0});
        } else if (cmd == "tgt_tick" && read(in, a, 1)) {
            print({targeting.tick(static_cast<std::uint32_t>(a[0])) ? 1.0 : 0.0});
        } else if (cmd == "tgt_dump") {
            std::array<std::size_t, mission::kMaxTargets> order{};
            const std::size_t n = targeting.prioritised(order);
            for (std::size_t i = 0; i < n; ++i) {
                const mission::Target& t = targeting.target(order[i]);
                std::printf("%u %s %.17g\n", static_cast<unsigned>(t.id), icd::to_string(t.status), t.track_quality);
            }
            std::printf("end\n");
        } else if (cmd == "oob_clear") {
            oob.init();
            std::printf("ok\n");
        } else if (cmd == "oob_add" && read(in, a, 6)) {
            print({static_cast<double>(oob.add(static_cast<icd::UnitKind>(byte(a[0])),
                                               static_cast<icd::Affiliation>(byte(a[1])), icd::UnitStatus::OPERATIONAL,
                                               byte(a[5]), a[2], a[3], a[4]))});
        } else if (cmd == "oob_threats" && read(in, a, 1)) {
            std::array<std::size_t, mission::kMaxUnits> idx{};
            const std::size_t n = oob.threats(byte(a[0]), idx);
            for (std::size_t i = 0; i < n; ++i) {
                std::printf("%u\n", static_cast<unsigned>(oob.unit(idx[i]).id));
            }
            std::printf("end\n");
        } else if (cmd == "oob_summary") {
            print({static_cast<double>(oob.friendly()), static_cast<double>(oob.enemy()),
                   static_cast<double>(oob.neutral())});
        } else {
            std::printf("error unknown command\n");
        }
    }
    return 0;
}
