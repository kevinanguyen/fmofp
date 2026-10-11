// The flight management partition process: terminal RT 12 on the bus.
//
//   fmofp_fms [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (FMS_SENSORS,
// FMS_ATTITUDE_IN, FMS_NAV_SOLUTION and the six commands), run
// FlightManagement::step(), and load the outputs (FMS_STATUS,
// FMS_FLIGHT_DATA, FMS_ATTITUDE_STATUS, FMS_NAV_UPDATE_STATUS,
// FMS_MANEUVER_STATUS, FMS_CONTROL_STATUS, FMS_ENERGY, TERM_STATUS) for the
// bus controller to collect.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/fms/flight_management.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace icd = fmofp::icd;

constexpr std::uint8_t kRt = icd::FmsStatus::kRt;

struct Options {
    fmofp::rt::RtProcessConfig rt{};
    unsigned long syncs{0};
    bool quiet{false};
};

bool parse_number(const char* text, unsigned long max, unsigned long& out) {
    if (text == nullptr) {
        return false;
    }
    char* end = nullptr;
    const unsigned long v = std::strtoul(text, &end, 10);
    if (end == text || *end != '\0' || v > max) {
        return false;
    }
    out = v;
    return true;
}

bool parse(int argc, char** argv, Options& o) {
    o.rt.rt = kRt;
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const char* next = i + 1 < argc ? argv[i + 1] : nullptr;
        unsigned long v = 0;
        if (std::strcmp(a, "--port-base") == 0 && parse_number(next, 65000, v)) {
            o.rt.port_base = static_cast<std::uint16_t>(v);
            ++i;
        } else if (std::strcmp(a, "--syncs") == 0 && parse_number(next, 1000000000UL, v)) {
            o.syncs = v;
            ++i;
        } else if (std::strcmp(a, "--quiet") == 0) {
            o.quiet = true;
        } else {
            return false;
        }
    }
    return true;
}

template <typename Message>
bool take_fresh(fmofp::bus::RemoteTerminal& terminal, Message& out) {
    typename Message::Words words{};
    std::uint8_t count = 0;
    bool fresh = false;
    if (terminal.get_received(Message::kSubaddress, words.data(), Message::kWords, count, fresh) != Status::kOk ||
        !fresh || count != Message::kWords) {
        return false;
    }
    return Message::decode(words, out) == Status::kOk;
}

template <typename Message>
Status load(fmofp::bus::RemoteTerminal& terminal, const Message& message) {
    typename Message::Words words{};
    const Status s = message.encode(words);
    if (!fmofp::ok(s)) {
        return s;
    }
    return terminal.set_transmit(Message::kSubaddress, words.data(), Message::kWords);
}

Status load_outputs(fmofp::bus::RemoteTerminal& terminal, const fmofp::fms::FlightManagement& fm,
                    fmofp::bus::Word last_sync) {
    Status result = Status::kOk;
    result = fmofp::worst(result, load(terminal, fm.status()));
    result = fmofp::worst(result, load(terminal, fm.flight_data()));
    result = fmofp::worst(result, load(terminal, fm.attitude_status()));
    result = fmofp::worst(result, load(terminal, fm.nav_update_status()));
    result = fmofp::worst(result, load(terminal, fm.maneuver_status()));
    result = fmofp::worst(result, load(terminal, fm.control_status()));
    result = fmofp::worst(result, load(terminal, fm.energy()));

    icd::TermStatus ts{};
    ts.state = fm.state();
    ts.healthy = fm.status().healthy;
    ts.last_cmd_seq = fm.status().last_cmd_seq;
    ts.cmd_rejected = fm.status().cmd_rejected;
    ts.bit_in_progress = fm.bit().in_progress;
    ts.bit_failed = fm.bit().failed;
    ts.fault_count = fm.bit().fault_count;
    ts.icd_major = icd::kIcdMajor;
    ts.icd_minor = icd::kIcdMinor;
    ts.last_sync = last_sync;
    result = fmofp::worst(result, load(terminal, ts));
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr, "usage: fmofp_fms [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "fms: init failed: %s (port %u in use?)\n", fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + kRt));
        return 1;
    }
    fmofp::fms::FlightManagement fm;
    if (fm.init() != Status::kOk) {
        std::fprintf(stderr, "fms: logic init failed\n");
        return 1;
    }
    fmofp::bus::RemoteTerminal& terminal = process.terminal();
    (void)load_outputs(terminal, fm, 0);
    if (!o.quiet) {
        std::printf("fms: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        fmofp::bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                fm.reset();
                (void)load_outputs(terminal, fm, 0);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            fm.start_self_test();
        }
        terminal.set_bit(fm.bit());

        icd::FmsModeCmd mode_cmd{};
        if (take_fresh(terminal, mode_cmd)) {
            fm.command_mode(mode_cmd);
        }
        icd::FmsFlightModeCmd flight_mode_cmd{};
        if (take_fresh(terminal, flight_mode_cmd)) {
            fm.command_flight_mode(flight_mode_cmd);
        }
        icd::FmsAttitudeCmd attitude_cmd{};
        if (take_fresh(terminal, attitude_cmd)) {
            fm.command_attitude(attitude_cmd);
        }
        icd::FmsNavUpdateCmd nav_update_cmd{};
        if (take_fresh(terminal, nav_update_cmd)) {
            fm.command_nav_update(nav_update_cmd);
        }
        icd::FmsManeuverCmd maneuver_cmd{};
        if (take_fresh(terminal, maneuver_cmd)) {
            fm.command_maneuver(maneuver_cmd);
        }
        icd::FmsControlCmd control_cmd{};
        if (take_fresh(terminal, control_cmd)) {
            fm.command_control(control_cmd);
        }
        icd::FmsSensors sensors{};
        const bool have_sensors = take_fresh(terminal, sensors);
        icd::FmsAttitudeIn attitude{};
        const bool have_attitude = take_fresh(terminal, attitude);
        icd::FmsNavSolution nav{};
        const bool have_nav = take_fresh(terminal, nav);
        fm.step(have_sensors ? &sensors : nullptr, have_attitude ? &attitude : nullptr, have_nav ? &nav : nullptr,
                frame);
        terminal.set_subsystem_flag(fm.state() == icd::TerminalState::DEGRADED);
        (void)load_outputs(terminal, fm, sync);

        if (!o.quiet && syncs % 50 == 1) {
            std::uint16_t major = 0;
            std::uint8_t minor = 0;
            fmofp::bc::decode_sync_word(sync, major, minor);
            std::printf("fms: major frame %u, state %s, flight mode %s, energy %.1f%%\n", major,
                        icd::to_string(fm.state()), icd::to_string(fm.flight_mode()), fm.flight_data().energy_state);
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const fmofp::bus::RtCounters& c = terminal.counters();
    std::printf("fms: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u state=%s\n", syncs,
                c.commands, c.receives, c.transmits, c.message_errors, icd::to_string(fm.state()));
    return 0;
}
