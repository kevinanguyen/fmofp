// A stand-in terminal: answers the bus controller with a real TERM_STATUS
// message and otherwise does nothing. Used to bring up the bus before any
// subsystem is ported, and by the Phase 1 gate.
//
//   fmofp_rt_stub --rt 9 [--port-base 51000] [--syncs 0] [--quiet]
//
// --syncs N exits after N SYNCHRONIZE_WITH_DATA frames (0 = run until killed).

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;

struct Options {
    fmofp::rt::RtProcessConfig rt{};
    unsigned long syncs{0};
    bool quiet{false};
    bool have_rt{false};
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
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        unsigned long v = 0;
        if (std::strcmp(a, "--rt") == 0 && parse_number(i + 1 < argc ? argv[i + 1] : nullptr, 30, v)) {
            o.rt.rt = static_cast<std::uint8_t>(v);
            o.have_rt = true;
            ++i;
        } else if (std::strcmp(a, "--port-base") == 0 &&
                   parse_number(i + 1 < argc ? argv[i + 1] : nullptr, 65000, v)) {
            o.rt.port_base = static_cast<std::uint16_t>(v);
            ++i;
        } else if (std::strcmp(a, "--syncs") == 0 &&
                   parse_number(i + 1 < argc ? argv[i + 1] : nullptr, 1000000000UL, v)) {
            o.syncs = v;
            ++i;
        } else if (std::strcmp(a, "--quiet") == 0) {
            o.quiet = true;
        } else {
            return false;
        }
    }
    return o.have_rt;
}

// What this terminal reports about itself, refreshed every frame.
Status load_term_status(fmofp::bus::RemoteTerminal& terminal, fmofp::bus::Word last_sync,
                        fmofp::icd::TerminalState state) {
    fmofp::icd::TermStatus status{};
    status.state = state;
    status.healthy = true;
    status.icd_major = fmofp::icd::kIcdMajor;
    status.icd_minor = fmofp::icd::kIcdMinor;
    status.bit_failed = terminal.bit().failed;
    status.bit_in_progress = terminal.bit().in_progress;
    status.fault_count = terminal.bit().fault_count;
    status.last_sync = last_sync;
    fmofp::icd::TermStatus::Words words{};
    const Status s = status.encode(words);
    if (!fmofp::ok(s)) {
        return s;
    }
    return terminal.set_transmit(fmofp::icd::TermStatus::kSubaddress, words.data(),
                                 fmofp::icd::TermStatus::kWords);
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr, "usage: fmofp_rt_stub --rt N [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "rt %u: init failed: %s (port %u in use?)\n", o.rt.rt, fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + o.rt.rt));
        return 1;
    }
    if (!o.quiet) {
        std::printf("rt %u listening on 127.0.0.1:%u, replying to port %u\n", o.rt.rt,
                    static_cast<unsigned>(o.rt.port_base + o.rt.rt),
                    static_cast<unsigned>(o.rt.port_base + o.rt.bc_port_offset));
    }

    // Nothing has been sent yet, so the first status word says INITIALIZING;
    // NORMAL from the first sync on.
    (void)load_term_status(process.terminal(), 0, fmofp::icd::TerminalState::INITIALIZING);

    unsigned long syncs = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        fmofp::bus::Word sync = 0;
        if (process.terminal().take_sync(sync)) {
            ++syncs;
            (void)load_term_status(process.terminal(), sync, fmofp::icd::TerminalState::NORMAL);
            if (!o.quiet && syncs % 50 == 1) {
                std::uint16_t major = 0;
                std::uint8_t minor = 0;
                fmofp::bc::decode_sync_word(sync, major, minor);
                std::printf("rt %u: major frame %u (sync %lu)\n", o.rt.rt, major, syncs);
            }
            if (o.syncs != 0 && syncs >= o.syncs) {
                break;
            }
        }
        if (process.terminal().take_reset_request()) {
            (void)load_term_status(process.terminal(), 0, fmofp::icd::TerminalState::INITIALIZING);
        }
    }

    const fmofp::bus::RtCounters& c = process.terminal().counters();
    const fmofp::rt::RtProcessStats& s = process.stats();
    std::printf("rt %u: syncs=%lu commands=%u receives=%u transmits=%u broadcasts=%u message_errors=%u "
                "datagrams=%u replies=%u bad_frames=%u send_failures=%u\n",
                o.rt.rt, syncs, c.commands, c.receives, c.transmits, c.broadcasts, c.message_errors, s.datagrams,
                s.replies, s.bad_frames, s.send_failures);
    return 0;
}
