// The navigation partition process: terminal RT 7 on the bus.
//
//   fmofp_nav [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (NAV_SENSORS, NAV_GPS,
// TERM_CMD), run Navigation::step(), and load NAV_SOLUTION and TERM_STATUS for
// the bus controller to collect.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>
#include <fmofp/nav/navigation.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace icd = fmofp::icd;

constexpr std::uint8_t kRt = icd::NavSolution::kRt;

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

Status load_outputs(fmofp::bus::RemoteTerminal& terminal, const fmofp::nav::Navigation& nav,
                    fmofp::bus::Word last_sync) {
    Status result = load(terminal, nav.solution());
    icd::TermStatus ts{};
    ts.state = nav.state();
    ts.healthy = nav.healthy();
    ts.last_cmd_seq = nav.last_cmd_seq();
    ts.cmd_rejected = nav.cmd_rejected();
    ts.bit_in_progress = nav.bit().in_progress;
    ts.bit_failed = nav.bit().failed;
    ts.fault_count = nav.bit().fault_count;
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
        std::fprintf(stderr, "usage: fmofp_nav [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "nav: init failed: %s (port %u in use?)\n", fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + kRt));
        return 1;
    }
    static fmofp::nav::Navigation nav;  // holds the constellation and the solver state; keep it off the stack
    if (nav.init() != Status::kOk) {
        std::fprintf(stderr, "nav: logic init failed\n");
        return 1;
    }
    fmofp::bus::RemoteTerminal& terminal = process.terminal();
    (void)load_outputs(terminal, nav, 0);
    if (!o.quiet) {
        std::printf("nav: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        fmofp::bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                nav.reset();
                (void)load_outputs(terminal, nav, 0);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            nav.start_self_test();
        }
        terminal.set_bit(nav.bit());

        icd::TermCmd cmd{};
        if (take_fresh(terminal, cmd)) {
            nav.command(cmd);
        }
        icd::NavSensors sensors{};
        const bool have_sensors = take_fresh(terminal, sensors);
        icd::NavGps gps{};
        const bool have_gps = take_fresh(terminal, gps);
        nav.step(have_sensors ? &sensors : nullptr, have_gps ? &gps : nullptr, frame);
        terminal.set_subsystem_flag(nav.state() == icd::TerminalState::DEGRADED);
        (void)load_outputs(terminal, nav, sync);

        if (!o.quiet && syncs % 50 == 1) {
            std::uint16_t major = 0;
            std::uint8_t minor = 0;
            fmofp::bc::decode_sync_word(sync, major, minor);
            const icd::NavSolution& s = nav.solution();
            std::printf("nav: major frame %u, state %s, gps %s, fixes %u, %.5f %.5f %.0f ft\n", major,
                        icd::to_string(nav.state()), s.gps_valid ? "valid" : "no fix", s.fix_count, s.latitude,
                        s.longitude, s.altitude);
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const fmofp::bus::RtCounters& c = terminal.counters();
    std::printf("nav: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u state=%s fixes=%u\n", syncs,
                c.commands, c.receives, c.transmits, c.message_errors, icd::to_string(nav.state()),
                nav.solution().fix_count);
    return 0;
}
