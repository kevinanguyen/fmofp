// The mission computer partition process: terminal RT 6 on the bus.
//
//   fmofp_mission [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (MSN_MODE_CMD, MSN_CMD,
// the FMS flight data heard by broadcast), run MissionComputer::step(), load
// MSN_STATUS and TERM_STATUS, and serve the MSN_PLAN block (republished when
// the plan changes). The status and the plan reach the MFD and the TSD by
// broadcast.

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/block.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>
#include <fmofp/mission/mission_computer.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace icd = fmofp::icd;
namespace bus = fmofp::bus;

constexpr std::uint8_t kRt = icd::MsnStatus::kRt;

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
bool take_fresh(bus::RemoteTerminal& terminal, Message& out) {
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
Status load(bus::RemoteTerminal& terminal, const Message& message) {
    typename Message::Words words{};
    const Status s = message.encode(words);
    if (!fmofp::ok(s)) {
        return s;
    }
    return terminal.set_transmit(Message::kSubaddress, words.data(), Message::kWords);
}

Status load_outputs(bus::RemoteTerminal& terminal, const fmofp::mission::MissionComputer& mc, bus::Word last_sync) {
    Status result = load(terminal, mc.status());
    icd::TermStatus ts{};
    ts.state = mc.state();
    ts.healthy = mc.status().healthy;
    ts.last_cmd_seq = mc.status().last_cmd_seq;
    ts.cmd_rejected = mc.status().cmd_rejected;
    ts.bit_in_progress = mc.bit().in_progress;
    ts.bit_failed = mc.bit().failed;
    ts.fault_count = mc.bit().fault_count;
    ts.icd_major = icd::kIcdMajor;
    ts.icd_minor = icd::kIcdMinor;
    ts.last_sync = last_sync;
    result = fmofp::worst(result, load(terminal, ts));
    return result;
}

std::array<bus::Word, icd::MsnPlan::kMaxPayloadWords> g_plan_payload{};
std::array<bus::Word, icd::MsnPlan::kMaxPayloadWords> g_plan_storage{};  // the publisher's own copy, in flight
fmofp::mission::MissionComputer g_mission;

Status init_plan_block(bus::BlockPublisher& block) {
    return block.init(g_plan_storage.data(), static_cast<std::uint16_t>(g_plan_storage.size()));
}

void publish_plan(const bus::RemoteTerminal& terminal, bus::BlockPublisher& block) {
    std::uint16_t length = 0;
    const Status s = g_mission.plan().encode(g_plan_payload.data(), static_cast<std::uint16_t>(g_plan_payload.size()),
                                             length);
    if (s != Status::kOk && s != Status::kClamped) {
        return;
    }
    (void)block.publish(terminal, icd::MsnPlan::kSegmentSubaddress, g_plan_payload.data(), length);
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr, "usage: fmofp_mission [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "mission: init failed: %s (port %u in use?)\n", fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + kRt));
        return 1;
    }
    if (g_mission.init() != Status::kOk) {
        std::fprintf(stderr, "mission: logic init failed\n");
        return 1;
    }
    bus::RemoteTerminal& terminal = process.terminal();
    // The route's progress is measured against the FMS flight data broadcast (ICD README section 8).
    if (terminal.accept_broadcast(icd::BcastFmsFlightData::kSubaddress) != Status::kOk) {
        std::fprintf(stderr, "mission: cannot accept the flight data broadcast\n");
        return 1;
    }
    bus::BlockPublisher plan_block;
    if (init_plan_block(plan_block) != Status::kOk) {
        std::fprintf(stderr, "mission: plan block init failed\n");
        return 1;
    }
    if (g_mission.take_plan_update()) {
        publish_plan(terminal, plan_block);
    }
    (void)plan_block.serve(terminal, icd::MsnPlan::kDescriptorSubaddress, icd::MsnPlan::kSegmentSubaddress);
    (void)load_outputs(terminal, g_mission, 0);
    if (!o.quiet) {
        std::printf("mission: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    unsigned plans = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                g_mission.reset();
                plan_block = bus::BlockPublisher{};
                (void)init_plan_block(plan_block);
                if (g_mission.take_plan_update()) {
                    publish_plan(terminal, plan_block);
                }
                (void)plan_block.serve(terminal, icd::MsnPlan::kDescriptorSubaddress, icd::MsnPlan::kSegmentSubaddress);
                (void)load_outputs(terminal, g_mission, 0);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            g_mission.start_self_test();
        }
        terminal.set_bit(g_mission.bit());

        icd::MsnModeCmd mode_cmd{};
        if (take_fresh(terminal, mode_cmd)) {
            g_mission.command_mode(mode_cmd);
        }
        icd::MsnCmd cmd{};
        if (take_fresh(terminal, cmd)) {
            g_mission.command(cmd);
        }
        icd::BcastFmsFlightData flight{};
        const bool have_flight = take_fresh(terminal, flight);
        g_mission.step(have_flight ? &flight : nullptr, frame);
        if (g_mission.take_plan_update()) {
            publish_plan(terminal, plan_block);
            ++plans;
        }
        (void)plan_block.serve(terminal, icd::MsnPlan::kDescriptorSubaddress, icd::MsnPlan::kSegmentSubaddress);
        terminal.set_subsystem_flag(g_mission.state() == icd::TerminalState::DEGRADED);
        (void)load_outputs(terminal, g_mission, sync);

        if (!o.quiet && syncs % 50 == 1) {
            std::uint16_t major = 0;
            std::uint8_t minor = 0;
            fmofp::bc::decode_sync_word(sync, major, minor);
            const icd::MsnStatus& st = g_mission.status();
            std::printf("mission: major frame %u, state %s, phase %s, waypoint %u of %u (%.1f nm at %.0f), %u targets, "
                        "units %u/%u/%u, plan %u\n",
                        major, icd::to_string(g_mission.state()), icd::to_string(st.phase), st.active_index,
                        st.waypoint_count, st.distance_to_next, st.bearing_to_next, st.target_count, st.friendly_units,
                        st.enemy_units, st.neutral_units, plans);
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const bus::RtCounters& c = terminal.counters();
    std::printf("mission: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u state=%s plans=%u\n", syncs,
                c.commands, c.receives, c.transmits, c.message_errors, icd::to_string(g_mission.state()), plans);
    return 0;
}
