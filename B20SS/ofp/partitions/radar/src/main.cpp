// The radar computer process: terminal RT 9 on the bus, with the weather radar
// ported (the other four radars are still Python).
//
//   fmofp_radar [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (RDR_WX_MODE_CMD, the
// relayed RDR_WX_WEATHER), run WeatherRadar::step(), and load the outputs:
// RDR_WX_STATUS, RDR_WX_SCAN, TERM_STATUS, and the RDR_WX_PRODUCTS block,
// whose descriptor and next segment sit on their subaddresses for the bus
// controller to collect. Each time the controller has read the segment
// subaddress, the next segment is loaded.

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
#include <fmofp/radar/weather_radar.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace bus = fmofp::bus;
namespace icd = fmofp::icd;
using Products = icd::RdrWxProducts;

constexpr std::uint8_t kRt = icd::RdrWxStatus::kRt;

// Eight megabytes of fixed arrays: static storage, never the stack.
fmofp::radar::WeatherRadar g_radar;
std::array<bus::Word, Products::kMaxPayloadWords> g_block_storage{};
std::array<bus::Word, Products::kMaxPayloadWords> g_payload{};

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

Status load_block(bus::RemoteTerminal& terminal, const bus::BlockTransmitter& tx) {
    std::array<bus::Word, bus::kBlockDescriptorWords> descriptor{};
    tx.descriptor(descriptor.data());
    Status result = terminal.set_transmit(Products::kDescriptorSubaddress, descriptor.data(), bus::kBlockDescriptorWords);
    std::array<bus::Word, bus::kBlockSegmentWords> segment{};
    tx.next_segment(segment.data());
    result = fmofp::worst(result, terminal.set_transmit(Products::kSegmentSubaddress, segment.data(),
                                                        bus::kBlockSegmentWords));
    return result;
}

Status load_outputs(bus::RemoteTerminal& terminal, const fmofp::radar::WeatherRadar& radar, bus::Word last_sync) {
    Status result = Status::kOk;
    result = fmofp::worst(result, load(terminal, radar.status()));
    result = fmofp::worst(result, load(terminal, radar.scan_summary()));

    icd::TermStatus ts{};
    ts.state = radar.state();
    ts.healthy = radar.status().healthy;
    ts.last_cmd_seq = radar.status().last_cmd_seq;
    ts.cmd_rejected = radar.status().cmd_rejected;
    ts.bit_in_progress = radar.bit().in_progress;
    ts.bit_failed = radar.bit().failed;
    ts.fault_count = radar.bit().fault_count;
    ts.icd_major = icd::kIcdMajor;
    ts.icd_minor = icd::kIcdMinor;
    ts.last_sync = last_sync;
    result = fmofp::worst(result, load(terminal, ts));
    return result;
}

// Encodes the current products and publishes them as a new block version.
void publish_products(const bus::RemoteTerminal& terminal, fmofp::radar::WeatherRadar& radar,
                      bus::BlockTransmitter& tx,
                      std::uint32_t& segment_reads) {
    std::uint16_t length = 0;
    const Status s = radar.products().encode(g_payload.data(), static_cast<std::uint16_t>(g_payload.size()), length);
    if (s != Status::kOk && s != Status::kClamped) {
        return;
    }
    if (tx.set_payload(g_payload.data(), length) != Status::kOk) {
        return;
    }
    radar.set_products_version(tx.version());
    segment_reads = terminal.transmit_count(Products::kSegmentSubaddress);
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr, "usage: fmofp_radar [--port-base N] [--syncs N] [--quiet]\n");
        return 2;
    }

    fmofp::rt::RtProcess process;
    const Status init = process.init(o.rt);
    if (init != Status::kOk) {
        std::fprintf(stderr, "radar: init failed: %s (port %u in use?)\n", fmofp::to_string(init),
                     static_cast<unsigned>(o.rt.port_base + kRt));
        return 1;
    }
    bus::BlockTransmitter tx;
    if (g_radar.init() != Status::kOk ||
        tx.init(g_block_storage.data(), static_cast<std::uint16_t>(g_block_storage.size())) != Status::kOk) {
        std::fprintf(stderr, "radar: logic init failed\n");
        return 1;
    }
    bus::RemoteTerminal& terminal = process.terminal();
    (void)load_outputs(terminal, g_radar, 0);
    (void)load_block(terminal, tx);
    if (!o.quiet) {
        std::printf("radar: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    std::uint32_t segment_reads = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                g_radar.reset();
                (void)tx.init(g_block_storage.data(), static_cast<std::uint16_t>(g_block_storage.size()));
                segment_reads = 0;
                (void)load_outputs(terminal, g_radar, 0);
                (void)load_block(terminal, tx);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            g_radar.start_self_test();
        }
        terminal.set_bit(g_radar.bit());

        icd::RdrWxModeCmd mode_cmd{};
        if (take_fresh(terminal, mode_cmd)) {
            g_radar.command_mode(mode_cmd);
        }
        icd::RdrWxWeather weather{};
        const bool have_weather = take_fresh(terminal, weather);
        g_radar.step(have_weather ? &weather : nullptr, frame);

        if (g_radar.take_products_update()) {
            publish_products(terminal, g_radar, tx, segment_reads);
        } else if (terminal.transmit_count(Products::kSegmentSubaddress) != segment_reads) {
            segment_reads = terminal.transmit_count(Products::kSegmentSubaddress);
            tx.advance();
        }
        (void)load_block(terminal, tx);
        terminal.set_subsystem_flag(g_radar.state() == icd::TerminalState::DEGRADED);
        (void)load_outputs(terminal, g_radar, sync);

        if (!o.quiet && syncs % 50 == 1) {
            std::uint16_t major = 0;
            std::uint8_t minor = 0;
            fmofp::bc::decode_sync_word(sync, major, minor);
            const icd::RdrWxScan& scan = g_radar.scan_summary();
            std::printf("radar: major frame %u, state %s, mode %s, scan %u (%s), max %.1f dBZ, %u cells, %u VIL points, "
                        "block v%u\n",
                        major, icd::to_string(g_radar.state()), icd::to_string(g_radar.mode()), scan.scan_id,
                        icd::to_string(scan.mode), scan.max_reflectivity, scan.storm_cells, scan.vil_points,
                        scan.products_version);
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const bus::RtCounters& c = terminal.counters();
    std::printf("radar: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u scans=%u state=%s\n", syncs,
                c.commands, c.receives, c.transmits, c.message_errors, g_radar.scans_completed(),
                icd::to_string(g_radar.state()));
    return 0;
}
