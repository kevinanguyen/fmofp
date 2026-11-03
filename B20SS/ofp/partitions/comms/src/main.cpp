// The communications partition process: terminal RT 2 on the bus.
//
//   fmofp_comms [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (COM_MODE_CMD, COM_CMD,
// the plant's RF environment relayed as COM_RF), run CommsComputer::step(), and
// load COM_STATUS, COM_LINK_STATUS and TERM_STATUS for the bus controller to
// collect. The link status reaches the MFD's communications page by relay.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/comms/comms_computer.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace icd = fmofp::icd;

constexpr std::uint8_t kRt = icd::ComStatus::kRt;

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

Status load_outputs(fmofp::bus::RemoteTerminal& terminal, const fmofp::comms::CommsComputer& cc,
                    fmofp::bus::Word last_sync) {
    Status result = load(terminal, cc.status());
    result = fmofp::worst(result, load(terminal, cc.link_status()));
    icd::TermStatus ts{};
    ts.state = cc.state();
    ts.healthy = cc.status().healthy;
    ts.last_cmd_seq = cc.status().last_cmd_seq;
    ts.cmd_rejected = cc.status().cmd_rejected;
    ts.bit_in_progress = cc.bit().in_progress;
    ts.bit_failed = cc.bit().failed;
    ts.fault_count = cc.bit().fault_count;
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
        std::fprintf(stderr, "usage: fmofp_comms [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "comms: init failed: %s (port %u in use?)\n", fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + kRt));
        return 1;
    }
    fmofp::comms::CommsComputer cc;
    if (cc.init() != Status::kOk) {
        std::fprintf(stderr, "comms: logic init failed\n");
        return 1;
    }
    fmofp::bus::RemoteTerminal& terminal = process.terminal();
    (void)load_outputs(terminal, cc, 0);
    if (!o.quiet) {
        std::printf("comms: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        fmofp::bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                cc.reset();
                (void)load_outputs(terminal, cc, 0);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            cc.start_self_test();
        }
        terminal.set_bit(cc.bit());

        icd::ComModeCmd mode_cmd{};
        if (take_fresh(terminal, mode_cmd)) {
            cc.command_mode(mode_cmd);
        }
        icd::ComCmd cmd{};
        if (take_fresh(terminal, cmd)) {
            cc.command(cmd);
        }
        icd::ComRf rf{};
        const bool have_rf = take_fresh(terminal, rf);
        cc.step(have_rf ? &rf : nullptr, frame);
        terminal.set_subsystem_flag(cc.state() == icd::TerminalState::DEGRADED);
        (void)load_outputs(terminal, cc, sync);

        if (!o.quiet && syncs % 50 == 1) {
            std::uint16_t major = 0;
            std::uint8_t minor = 0;
            fmofp::bc::decode_sync_word(sync, major, minor);
            const icd::ComLinkStatus& l = cc.link_status();
            std::printf("comms: major frame %u, state %s, radio %.3f MHz %s %.0f%%, satcom %s, data link %s (%u/%u)\n",
                        major, icd::to_string(cc.state()), l.frequency, l.radio_active ? "ACTIVE" : "SQUELCH",
                        l.radio_signal, icd::to_string(l.satcom_state), icd::to_string(l.datalink_state),
                        l.packets_sent, l.packets_received);
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const fmofp::bus::RtCounters& c = terminal.counters();
    std::printf("comms: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u state=%s\n", syncs,
                c.commands, c.receives, c.transmits, c.message_errors, icd::to_string(cc.state()));
    return 0;
}
