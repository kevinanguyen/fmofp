// fmofp_engine_tool: the engine partition's pure functions on stdin/stdout,
// so the Python tests can run the original thrustManagement.py beside them
// (B20SS/fmofp_bus/tests/test_engine_oracle.py). Test support, not flight code.
//
// One command per line; every number is printed with 17 significant digits.
//   set_all <pct>                 -> t1 t2
//   set <engine 1|2> <pct>        -> t1 t2
//   engage <target_kt>            -> t1 t2
//   disengage                     -> t1 t2
//   update <airspeed_kt> <dt>     -> t1 t2 integral last_error
//   asym                          -> asymmetry asymmetric(0|1)
//   fuel_init                     -> left right center total
//   fuel_step <ff1_kgh> <ff2_kgh> <dt> -> left right center total burn_kgh low_bits critical_bits leak
//   fuel_set <tank 0-2> <kg>      -> left right center
//   fuel_transfer <src> <dst> <kg> -> moved left right center

#include <cstdio>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <fmofp/engine/engine_management.hpp>

namespace {

namespace engine = fmofp::engine;

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

double bits(const std::array<engine::TankFlags, engine::kTanks>& flags, bool engine::TankFlags::*member) {
    double value = 0.0;
    double weight = 1.0;
    for (const engine::TankFlags& f : flags) {
        if (f.*member) {
            value += weight;
        }
        weight *= 2.0;
    }
    return value;
}

}  // namespace

int main() {
    engine::ThrustManager thrust;
    engine::FuelSystem fuel;
    fuel.init();
    std::string line;
    std::vector<double> a;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd) || cmd.empty() || cmd[0] == '#') {
            continue;
        }
        if (cmd == "set_all" && read(in, a, 1)) {
            thrust.set_all(a[0]);
            print({thrust.thrust[0], thrust.thrust[1]});
        } else if (cmd == "set" && read(in, a, 2)) {
            thrust.set_thrust(static_cast<std::size_t>(a[0]) - 1U, a[1]);
            print({thrust.thrust[0], thrust.thrust[1]});
        } else if (cmd == "engage" && read(in, a, 1)) {
            thrust.engage(a[0]);
            print({thrust.thrust[0], thrust.thrust[1]});
        } else if (cmd == "disengage") {
            thrust.disengage();
            print({thrust.thrust[0], thrust.thrust[1]});
        } else if (cmd == "update" && read(in, a, 2)) {
            thrust.update_autothrottle(a[0], a[1]);
            print({thrust.thrust[0], thrust.thrust[1], thrust.integral, thrust.last_error});
        } else if (cmd == "asym") {
            print({thrust.asymmetry(), thrust.asymmetric() ? 1.0 : 0.0});
        } else if (cmd == "fuel_init") {
            fuel.init();
            print({fuel.level(0), fuel.level(1), fuel.level(2), fuel.total()});
        } else if (cmd == "fuel_step" && read(in, a, 3)) {
            fuel.step({a[0], a[1]}, a[2]);
            print({fuel.level(0), fuel.level(1), fuel.level(2), fuel.total(), fuel.burn_rate_kgh(),
                   bits(fuel.flags(), &engine::TankFlags::low), bits(fuel.flags(), &engine::TankFlags::critical),
                   bits(fuel.flags(), &engine::TankFlags::leak)});
        } else if (cmd == "fuel_set" && read(in, a, 2)) {
            fuel.set_level(static_cast<std::size_t>(a[0]), a[1]);
            print({fuel.level(0), fuel.level(1), fuel.level(2)});
        } else if (cmd == "fuel_transfer" && read(in, a, 3)) {
            const double moved = fuel.transfer(static_cast<std::size_t>(a[0]), static_cast<std::size_t>(a[1]), a[2]);
            print({moved, fuel.level(0), fuel.level(1), fuel.level(2)});
        } else {
            std::printf("error unknown command\n");
        }
    }
    return 0;
}
