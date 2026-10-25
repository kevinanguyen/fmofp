// The bus controller process.
//
//   fmofp_bc --rt 1 --rt 5 [--port-base 51000] [--frames 0] [--free-run]
//            [--no-monitor] [--timeout-ms 5] [--report] [--strict] [--script FILE]
//
// --frames N runs N minor frames (0 = until killed). --report prints one line
// per major frame. --strict exits 1 if any terminal failed to answer or any
// frame overran, which is how the Phase 1 gate is scored. --script FILE sends
// commands at given frames: one per line, "frame rt sa word word ..." with
// the data words in hex (lines starting with # are comments). That is how the
// tests command a partition over the bus.

#include <array>
#include <cstddef>
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
    bool print_schedule{false};
    bool strict{false};
    const char* script{nullptr};
};

// A command from --script: data to a terminal in a given minor frame.
struct ScriptedCommand {
    unsigned long frame{0};
    std::uint8_t rt{0};
    std::uint8_t sa{0};
    std::uint8_t count{0};
    std::array<fmofp::bus::Word, fmofp::bus::kMaxDataWords> words{};
};

constexpr std::size_t kMaxScriptedCommands = 128;
constexpr std::size_t kMaxScriptLine = 512;

std::array<ScriptedCommand, kMaxScriptedCommands> g_script{};
std::size_t g_script_count = 0;

bool parse_field(char*& cursor, int base, unsigned long max, unsigned long& out) {
    char* end = nullptr;
    const unsigned long v = std::strtoul(cursor, &end, base);
    if (end == cursor || v > max) {
        return false;
    }
    cursor = end;
    out = v;
    return true;
}

// Reads the script; false with a message on stderr if a line is malformed.
bool load_script(const char* path) {
    std::FILE* f = std::fopen(path, "r");
    if (f == nullptr) {
        std::fprintf(stderr, "bc: cannot open script %s\n", path);
        return false;
    }
    char line[kMaxScriptLine];
    unsigned line_number = 0;
    bool ok = true;
    while (ok && std::fgets(line, sizeof line, f) != nullptr) {
        ++line_number;
        char* cursor = line;
        while (*cursor == ' ' || *cursor == '\t') {
            ++cursor;
        }
        if (*cursor == '\0' || *cursor == '\n' || *cursor == '\r' || *cursor == '#') {
            continue;
        }
        if (g_script_count >= kMaxScriptedCommands) {
            std::fprintf(stderr, "bc: script has more than %zu commands\n", kMaxScriptedCommands);
            ok = false;
            break;
        }
        ScriptedCommand& c = g_script[g_script_count];
        unsigned long frame = 0;
        unsigned long rt = 0;
        unsigned long sa = 0;
        ok = parse_field(cursor, 10, 1000000000UL, frame) && parse_field(cursor, 10, 30, rt) &&
             parse_field(cursor, 10, 30, sa) && sa >= 1;
        c.frame = frame;
        c.rt = static_cast<std::uint8_t>(rt);
        c.sa = static_cast<std::uint8_t>(sa);
        c.count = 0;
        while (ok) {
            while (*cursor == ' ' || *cursor == '\t') {
                ++cursor;
            }
            if (*cursor == '\0' || *cursor == '\n' || *cursor == '\r' || *cursor == '#') {
                break;
            }
            unsigned long word = 0;
            if (c.count >= fmofp::bus::kMaxDataWords || !parse_field(cursor, 16, 0xFFFFUL, word)) {
                ok = false;
                break;
            }
            c.words[c.count] = static_cast<fmofp::bus::Word>(word);
            ++c.count;
        }
        ok = ok && c.count >= 1;
        if (!ok) {
            std::fprintf(stderr, "bc: script %s line %u: expected 'frame rt sa hexword...'\n", path, line_number);
            break;
        }
        ++g_script_count;
    }
    (void)std::fclose(f);
    return ok;
}

// Queues the script's commands for `frame`. Returns how many were queued.
unsigned queue_scripted(fmofp::bc::BusController& bc, unsigned long frame) {
    unsigned queued = 0;
    for (std::size_t i = 0; i < g_script_count; ++i) {
        const ScriptedCommand& c = g_script[i];
        if (c.frame != frame) {
            continue;
        }
        const Status s = bc.queue_receive(c.rt, c.sa, c.words.data(), c.count);
        if (s != Status::kOk) {
            std::fprintf(stderr, "bc: frame %lu: could not queue scripted command to RT %u SA %u: %s\n", frame,
                         c.rt, c.sa, fmofp::to_string(s));
        } else {
            ++queued;
        }
    }
    return queued;
}

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
        } else if (std::strcmp(a, "--schedule") == 0) {
            o.print_schedule = true;
        } else if (std::strcmp(a, "--script") == 0 && next != nullptr) {
            o.script = next;
            ++i;
        } else {
            return false;
        }
    }
    return true;
}

// --schedule: one line per minor frame, its bus time and its slots as rt/sa, T or R, the
// word count, and each relay consumer after a '>'.
void print_schedule(const fmofp::bc::Schedule& schedule) {
    for (std::uint8_t minor = 0; minor < fmofp::bc::kMinorFramesPerMajor; ++minor) {
        const fmofp::bc::FrameSlots& f = schedule.frame(minor);
        std::printf("bc: frame %2u %5u us:", static_cast<unsigned>(minor), schedule.frame_load_us(minor));
        for (std::size_t i = 0; i < f.count; ++i) {
            const fmofp::bc::Slot& slot = f.slots[i];
            std::printf(" %u/%u%c%u", static_cast<unsigned>(slot.rt), static_cast<unsigned>(slot.subaddress),
                        slot.transmit ? 'T' : 'R', static_cast<unsigned>(slot.words));
            for (std::uint8_t r = 0; r < slot.relay_count; ++r) {
                std::printf(">%u/%u", static_cast<unsigned>(slot.relays[r].rt),
                            static_cast<unsigned>(slot.relays[r].subaddress));
            }
        }
        std::printf("\n");
    }
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
        std::printf("  rt %2u: transactions=%u retries=%u no_response=%u message_errors=%u malformed=%u busy=%u %s\n",
                    rt, t.transactions, t.retries, t.no_response, t.message_errors, t.malformed, t.busy,
                    t.responded ? "responding" : "NEVER ANSWERED");
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr,
                     "usage: fmofp_bc --rt N [--rt N ...] [--port-base N] [--frames N] [--free-run] "
                     "[--no-monitor] [--timeout-ms N] [--report] [--strict] [--script FILE] [--schedule]\n");
        return 2;
    }
    if (o.script != nullptr && !load_script(o.script)) {
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
    if (o.print_schedule) {
        print_schedule(schedule);
    }
    if (schedule.worst_frame_load_us() > fmofp::bc::kMinorFrameBudgetUs) {
        std::fprintf(stderr, "bc: schedule exceeds the 50%% bus load budget\n");
        return 1;
    }
    if (o.print_schedule) {
        return 0;
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

    unsigned long scripted_sent = 0;
    for (unsigned long frame = 0; o.frames == 0 || frame < o.frames; ++frame) {
        scripted_sent += queue_scripted(bc, frame);
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
    if (o.script != nullptr) {
        std::printf("bc: script: %lu commands sent of %zu\n", scripted_sent, g_script_count);
    }
    bool failed = bc.stats().overruns != 0;
    for (std::uint8_t i = 0; i < o.bc.terminal_count; ++i) {
        const fmofp::bc::TerminalStats& t = bc.terminal(o.bc.terminals[i]);
        failed = failed || t.no_response != 0 || t.malformed != 0;
    }
    bc.shutdown();
    return (o.strict && failed) ? 1 : 0;
}
