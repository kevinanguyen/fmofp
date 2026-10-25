// The engine management partition process: terminal RT 3 on the bus.
//
//   fmofp_engine [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (ENG_FMS_CONTROL, the
// FMS flight data heard by broadcast, ENG_ENGINE_IN, ENG_MODE_CMD, ENG_CMD),
// run EngineManagement::step(), and load the outputs (ENG_STATUS,
// ENG_THRUST_CMD, ENG_ENGINE_STATUS, ENG_FUEL_STATUS, TERM_STATUS) for the bus
// controller to collect. The thrust command reaches the plant's engines by
// relay; the engine status reaches the flight computer and the displays by
// broadcast.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/engine/engine_management.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace icd = fmofp::icd;

constexpr std::uint8_t kRt = icd::EngStatus::kRt;

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

Status load_outputs(fmofp::bus::RemoteTerminal& terminal, const fmofp::engine::EngineManagement& em,
                    fmofp::bus::Word last_sync) {
    Status result = Status::kOk;
    result = fmofp::worst(result, load(terminal, em.status()));
    result = fmofp::worst(result, load(terminal, em.thrust_cmd()));
    result = fmofp::worst(result, load(terminal, em.engine_status()));
    result = fmofp::worst(result, load(terminal, em.fuel_status()));

    icd::TermStatus ts{};
    ts.state = em.state();
    ts.healthy = em.status().healthy;
    ts.last_cmd_seq = em.status().last_cmd_seq;
    ts.cmd_rejected = em.status().cmd_rejected;
    ts.bit_in_progress = em.bit().in_progress;
    ts.bit_failed = em.bit().failed;
    ts.fault_count = em.bit().fault_count;
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
        std::fprintf(stderr, "usage: fmofp_engine [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "engine: init failed: %s (port %u in use?)\n", fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + kRt));
        return 1;
    }
    fmofp::engine::EngineManagement em;
    if (em.init() != Status::kOk) {
        std::fprintf(stderr, "engine: logic init failed\n");
        return 1;
    }
    fmofp::bus::RemoteTerminal& terminal = process.terminal();
    // The airspeed the autothrottle holds comes with the FMS flight data broadcast (ICD README section 8).
    if (terminal.accept_broadcast(icd::BcastFmsFlightData::kSubaddress) != Status::kOk) {
        std::fprintf(stderr, "engine: cannot accept the flight data broadcast\n");
        return 1;
    }
    (void)load_outputs(terminal, em, 0);
    if (!o.quiet) {
        std::printf("engine: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        fmofp::bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                em.reset();
                (void)load_outputs(terminal, em, 0);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            em.start_self_test();
        }
        terminal.set_bit(em.bit());

        icd::EngModeCmd mode_cmd{};
        if (take_fresh(terminal, mode_cmd)) {
            em.command_mode(mode_cmd);
        }
        icd::EngCmd cmd{};
        if (take_fresh(terminal, cmd)) {
            em.command(cmd);
        }
        icd::EngFmsControl fms{};
        const bool have_fms = take_fresh(terminal, fms);
        icd::BcastFmsFlightData flight{};
        const bool have_flight = take_fresh(terminal, flight);
        icd::EngEngineIn engines{};
        const bool have_engines = take_fresh(terminal, engines);
        em.step(have_fms ? &fms : nullptr, have_flight ? &flight : nullptr, have_engines ? &engines : nullptr, frame);
        terminal.set_subsystem_flag(em.state() == icd::TerminalState::DEGRADED);
        (void)load_outputs(terminal, em, sync);

        if (!o.quiet && syncs % 50 == 1) {
            std::uint16_t major = 0;
            std::uint8_t minor = 0;
            fmofp::bc::decode_sync_word(sync, major, minor);
            std::printf("engine: major frame %u, state %s, thrust %.0f/%.0f%%, N1 %.1f/%.1f%%, fuel %.0f kg%s\n", major,
                        icd::to_string(em.state()), em.thrust_cmd().thrust_1, em.thrust_cmd().thrust_2,
                        em.engine_status().n1_1, em.engine_status().n1_2, em.fuel_status().total,
                        em.status().autothrottle_engaged ? ", autothrottle" : "");
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const fmofp::bus::RtCounters& c = terminal.counters();
    std::printf("engine: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u state=%s\n", syncs,
                c.commands, c.receives, c.transmits, c.message_errors, icd::to_string(em.state()));
    return 0;
}
