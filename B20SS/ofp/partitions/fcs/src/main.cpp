// The flight control partition process: terminal RT 5 on the bus.
//
//   fmofp_fcs [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (FCS_SENSORS,
// FCS_MODE_CMD, FCS_SURFACE_CMD), run FlightControl::step(), and load the
// outputs (FCS_STATUS, FCS_SURFACE_STATUS, FCS_ATTITUDE, FCS_GCAS_ALERTS,
// FCS_PERF_STATUS, TERM_STATUS) for the bus controller to collect.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/fcs/flight_control.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace icd = fmofp::icd;

constexpr std::uint8_t kRt = icd::FcsStatus::kRt;

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

// Reads a received message if one has arrived since the last frame.
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

Status load_outputs(fmofp::bus::RemoteTerminal& terminal, const fmofp::fcs::FlightControl& fc,
                    fmofp::bus::Word last_sync) {
    Status worst = Status::kOk;
    worst = fmofp::worst(worst, load(terminal, fc.status()));
    worst = fmofp::worst(worst, load(terminal, fc.surfaces()));
    worst = fmofp::worst(worst, load(terminal, fc.attitude()));
    worst = fmofp::worst(worst, load(terminal, fc.gcas()));
    worst = fmofp::worst(worst, load(terminal, fc.perf()));

    icd::TermStatus ts{};
    ts.state = fc.state();
    ts.healthy = fc.status().healthy;
    ts.last_cmd_seq = fc.status().last_cmd_seq;
    ts.cmd_rejected = fc.status().cmd_rejected;
    ts.bit_in_progress = fc.bit().in_progress;
    ts.bit_failed = fc.bit().failed;
    ts.fault_count = fc.bit().fault_count;
    ts.icd_major = icd::kIcdMajor;
    ts.icd_minor = icd::kIcdMinor;
    ts.last_sync = last_sync;
    worst = fmofp::worst(worst, load(terminal, ts));
    return worst;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr, "usage: fmofp_fcs [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "fcs: init failed: %s (port %u in use?)\n", fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + kRt));
        return 1;
    }
    fmofp::fcs::FlightControl fc;
    if (fc.init() != Status::kOk) {
        std::fprintf(stderr, "fcs: logic init failed\n");
        return 1;
    }
    fmofp::bus::RemoteTerminal& terminal = process.terminal();
    (void)load_outputs(terminal, fc, 0);
    if (!o.quiet) {
        std::printf("fcs: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        fmofp::bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                fc.reset();
                (void)load_outputs(terminal, fc, 0);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            fc.start_self_test();
        }
        terminal.set_bit(fc.bit());

        icd::FcsModeCmd mode_cmd{};
        if (take_fresh(terminal, mode_cmd)) {
            fc.command_mode(mode_cmd);
        }
        icd::FcsSurfaceCmd surface_cmd{};
        if (take_fresh(terminal, surface_cmd)) {
            fc.command_surface(surface_cmd);
        }
        icd::FcsSensors sensors{};
        const bool have_sensors = take_fresh(terminal, sensors);
        fc.step(have_sensors ? &sensors : nullptr, frame);
        terminal.set_subsystem_flag(fc.state() == icd::TerminalState::DEGRADED);
        (void)load_outputs(terminal, fc, sync);

        if (!o.quiet && syncs % 50 == 1) {
            std::uint16_t major = 0;
            std::uint8_t minor = 0;
            fmofp::bc::decode_sync_word(sync, major, minor);
            std::printf("fcs: major frame %u, state %s, gcas %s\n", major, icd::to_string(fc.state()),
                        icd::to_string(fc.gcas().status));
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const fmofp::bus::RtCounters& c = terminal.counters();
    std::printf("fcs: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u state=%s\n", syncs,
                c.commands, c.receives, c.transmits, c.message_errors, icd::to_string(fc.state()));
    return 0;
}
