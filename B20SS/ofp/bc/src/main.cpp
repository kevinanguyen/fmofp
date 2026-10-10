// The bus controller process.
//
//   fmofp_bc --rt 1 --rt 5 [--port-base 51000] [--frames 0] [--free-run]
//            [--no-monitor] [--timeout-ms 5] [--report] [--strict]
//
// --frames N runs N minor frames (0 = until killed). --report prints one line
// per major frame. --strict exits 1 if any terminal failed to answer or any
// frame overran, which is how the Phase 1 gate is scored.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/bus_controller.hpp>
#include <fmofp/bc/icd_schedule.hpp>
#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>

namespace {

using fmofp::Status;

struct Options {
    fmofp::bc::BcConfig bc{};
    unsigned long frames{0};
    bool report{false};
    bool strict{false};
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
        const char* next = i + 1 < argc ? argv[i + 1] : nullptr;
        unsigned long v = 0;
        if (std::strcmp(a, "--rt") == 0 && parse_number(next, 30, v)) {
            if (o.bc.terminal_count >= fmofp::bc::kMaxTerminals) {
                return false;
            }
            o.bc.terminals[o.bc.terminal_count++] = static_cast<std::uint8_t>(v);
            ++i;
        } else if (std::strcmp(a, "--port-base") == 0 && parse_number(next, 65000, v)) {
            o.bc.port_base = static_cast<std::uint16_t>(v);
            ++i;
        } else if (std::strcmp(a, "--frames") == 0 && parse_number(next, 1000000000UL, v)) {
            o.frames = v;
            ++i;
        } else if (std::strcmp(a, "--timeout-ms") == 0 && parse_number(next, 1000, v)) {
            o.bc.response_timeout = static_cast<fmofp::osal::Nanoseconds>(v) * fmofp::osal::kNsPerMs;
            ++i;
        } else if (std::strcmp(a, "--free-run") == 0) {
            o.bc.mode = fmofp::bc::BcMode::kFreeRun;
        } else if (std::strcmp(a, "--no-monitor") == 0) {
            o.bc.monitor_enabled = false;
        } else if (std::strcmp(a, "--report") == 0) {
            o.report = true;
        } else if (std::strcmp(a, "--strict") == 0) {
            o.strict = true;
        } else {
            return false;
        }
    }
    return true;
}

void print_stats(const fmofp::bc::BusController& bc) {
    const fmofp::bc::BcStats& s = bc.stats();
    std::printf("bc: frames=%u overruns=%u worst_frame_us=%lld aperiodic_sent=%u monitor_failures=%u\n", s.frames,
                s.overruns, static_cast<long long>(s.worst_frame_ns / fmofp::osal::kNsPerUs), s.aperiodic_sent,
                s.monitor_failures);
    const fmofp::bc::BcConfig& c = bc.config();
    for (std::uint8_t i = 0; i < c.terminal_count; ++i) {
        const std::uint8_t rt = c.terminals[i];
        const fmofp::bc::TerminalStats& t = bc.terminal(rt);
        std::printf("  rt %2u: transactions=%u no_response=%u message_errors=%u malformed=%u busy=%u %s\n", rt,
                    t.transactions, t.no_response, t.message_errors, t.malformed, t.busy,
                    t.responded ? "responding" : "NEVER ANSWERED");
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr,
                     "usage: fmofp_bc --rt N [--rt N ...] [--port-base N] [--frames N] [--free-run] "
                     "[--no-monitor] [--timeout-ms N] [--report] [--strict]\n");
        return 2;
    }

    fmofp::bc::Schedule schedule;
    Status s = fmofp::bc::add_icd_messages(schedule, o.bc.terminals.data(), o.bc.terminal_count);
    if (s == Status::kOk) {
        s = schedule.build();
    }
    if (s != Status::kOk) {
        std::fprintf(stderr, "bc: schedule failed: %s\n", fmofp::to_string(s));
        return 1;
    }
    std::printf("bc: %zu scheduled messages for %u terminals; worst minor frame %u us of %u us budget\n",
                schedule.message_count(), o.bc.terminal_count, schedule.worst_frame_load_us(),
                fmofp::bc::kMinorFrameBudgetUs);
    if (schedule.worst_frame_load_us() > fmofp::bc::kMinorFrameBudgetUs) {
        std::fprintf(stderr, "bc: schedule exceeds the 50%% bus load budget\n");
        return 1;
    }

    static fmofp::bc::BusController bc;  // ~80 KB of buffers; not for the stack
    s = bc.init(o.bc, schedule);
    if (s != Status::kOk) {
        std::fprintf(stderr, "bc: init failed: %s (port %u in use?)\n", fmofp::to_string(s),
                     static_cast<unsigned>(o.bc.port_base + o.bc.bc_port_offset));
        return 1;
    }
    std::printf("bc: %s on port base %u, %s\n",
                o.bc.mode == fmofp::bc::BcMode::kRealTime ? "real time, 20 ms minor frames" : "free running",
                o.bc.port_base, o.bc.monitor_enabled ? "monitor on" : "monitor off");

    for (unsigned long frame = 0; o.frames == 0 || frame < o.frames; ++frame) {
        const Status f = bc.run_minor_frame();
        if (f != Status::kOk && f != Status::kTimeout && f != Status::kMalformed) {
            std::fprintf(stderr, "bc: frame %lu failed: %s\n", frame, fmofp::to_string(f));
            break;
        }
        if (o.report && bc.minor_frame() == 0) {
            std::printf("bc: major frame %u done, overruns=%u worst_frame_us=%lld\n", bc.major_frame(),
                        bc.stats().overruns, static_cast<long long>(bc.stats().worst_frame_ns / fmofp::osal::kNsPerUs));
        }
    }

    print_stats(bc);
    bool failed = bc.stats().overruns != 0;
    for (std::uint8_t i = 0; i < o.bc.terminal_count; ++i) {
        const fmofp::bc::TerminalStats& t = bc.terminal(o.bc.terminals[i]);
        failed = failed || t.no_response != 0 || t.malformed != 0;
    }
    bc.shutdown();
    return (o.strict && failed) ? 1 : 0;
}
