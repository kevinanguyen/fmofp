// The radar computer process: terminal RT 9 on the bus, with the weather radar,
// the terrain-following radar, the targeting radar, the AEWC radar and the track
// fusion ported (SAR is still Python).
//
//   fmofp_radar [--port-base 51000] [--syncs 0] [--quiet]
//
// Each SYNCHRONIZE_WITH_DATA frame: take what arrived (RDR_WX_MODE_CMD and the
// relayed RDR_WX_WEATHER; RDR_TFR_MODE_CMD, the relayed RDR_TFR_SENSORS and the
// descriptor and segments of the relayed RDR_TFR_TERRAIN block; RDR_TGT_MODE_CMD
// and the relayed RDR_TRAFFIC block, which the AEWC radar reads too; RDR_AEWC_MODE_CMD),
// step the radars and the fusion, and load
// the outputs: RDR_WX_STATUS, RDR_WX_SCAN and the RDR_WX_PRODUCTS block;
// RDR_TFR_STATUS, RDR_TFR_GUIDANCE and the RDR_TFR_ELEVATION block;
// RDR_TGT_STATUS and the RDR_TGT_TRACKS block; RDR_AEWC_STATUS and the
// RDR_AEWC_TRACKS block; the RDR_FUSED_TRACKS block;
// TERM_STATUS for the computer as a whole. A block's descriptor and next
// segment sit on their subaddresses for the bus controller to collect; each
// time it has read the segment subaddress, the next segment is loaded.

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
#include <fmofp/radar/aewc_radar.hpp>
#include <fmofp/radar/targeting_radar.hpp>
#include <fmofp/radar/terrain_following.hpp>
#include <fmofp/radar/track_fusion.hpp>
#include <fmofp/radar/weather_radar.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
namespace bus = fmofp::bus;
namespace icd = fmofp::icd;
namespace radar = fmofp::radar;
using WxProducts = icd::RdrWxProducts;
using TfrProducts = icd::RdrTfrElevation;
using Terrain = icd::RdrTfrTerrain;
using TgtTracks = icd::RdrTgtTracks;
using FusedTracks = icd::RdrFusedTracks;
using AewcTracks = icd::RdrAewcTracks;
using Traffic = icd::RdrTraffic;

constexpr std::uint8_t kRt = icd::RdrWxStatus::kRt;

// Megabytes of fixed arrays: static storage, never the stack.
radar::WeatherRadar g_weather;
radar::TerrainFollowing g_tfr;
radar::TargetingRadar g_targeting;
radar::AewcRadar g_aewc;
radar::TrackFusion g_fusion;
std::array<bus::Word, WxProducts::kMaxPayloadWords> g_wx_storage{};
std::array<bus::Word, WxProducts::kMaxPayloadWords> g_wx_payload{};
std::array<bus::Word, TfrProducts::kMaxPayloadWords> g_tfr_storage{};
std::array<bus::Word, TfrProducts::kMaxPayloadWords> g_tfr_payload{};
std::array<bus::Word, Terrain::kMaxPayloadWords> g_terrain_storage{};
std::array<bus::Word, TgtTracks::kMaxPayloadWords> g_tgt_storage{};
std::array<bus::Word, TgtTracks::kMaxPayloadWords> g_tgt_payload{};
std::array<bus::Word, FusedTracks::kMaxPayloadWords> g_fused_storage{};
std::array<bus::Word, FusedTracks::kMaxPayloadWords> g_fused_payload{};
std::array<bus::Word, AewcTracks::kMaxPayloadWords> g_aewc_storage{};
std::array<bus::Word, AewcTracks::kMaxPayloadWords> g_aewc_payload{};
std::array<bus::Word, Traffic::kMaxPayloadWords> g_traffic_storage{};
std::array<radar::TrackReport, radar::kFusionMaxReports> g_reports{};

struct Options {
    fmofp::rt::RtProcessConfig rt{};
    unsigned long syncs{0};
    bool quiet{false};
};

// The block publishers and collectors the process serves.
struct Blocks {
    bus::BlockPublisher weather;
    bus::BlockPublisher elevation;
    bus::BlockPublisher tracks;
    bus::BlockPublisher fused;
    bus::BlockPublisher aewc;
    bus::BlockCollector terrain;
    bus::BlockCollector traffic;
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

// Which radar took the most recent command: TERM_STATUS reports its sequence and outcome.
enum class LastCommand : std::uint8_t { kNone, kWeather, kTfr, kTargeting, kAewc };

// The radar computer's state is the worse of its radars': a radar in trouble
// shows through; otherwise NORMAL if any is operating normally.
icd::TerminalState combined_state(icd::TerminalState a, icd::TerminalState b) {
    if (a == b) {
        return a;
    }
    for (const icd::TerminalState trouble :
         {icd::TerminalState::FAILURE, icd::TerminalState::EMERGENCY, icd::TerminalState::RECOVERY,
          icd::TerminalState::DEGRADED, icd::TerminalState::INITIALIZING}) {
        if (a == trouble || b == trouble) {
            return trouble;
        }
    }
    if (a == icd::TerminalState::NORMAL || b == icd::TerminalState::NORMAL) {
        return icd::TerminalState::NORMAL;
    }
    return a;
}

icd::TerminalState combined_state() {
    return combined_state(combined_state(combined_state(g_weather.state(), g_tfr.state()), g_targeting.state()),
                          g_aewc.state());
}

bus::BitWord combined_bit(const bus::BitWord& a, const bus::BitWord& b) {
    bus::BitWord c{};
    c.in_progress = a.in_progress || b.in_progress;
    c.failed = a.failed || b.failed;
    const unsigned faults = static_cast<unsigned>(a.fault_count) + static_cast<unsigned>(b.fault_count);
    c.fault_count = static_cast<std::uint8_t>(faults > 255U ? 255U : faults);
    return c;
}

bus::BitWord combined_bit() {
    return combined_bit(combined_bit(combined_bit(g_weather.bit(), g_tfr.bit()), g_targeting.bit()), g_aewc.bit());
}

bool any_degraded() {
    return g_weather.state() == icd::TerminalState::DEGRADED || g_tfr.state() == icd::TerminalState::DEGRADED ||
           g_targeting.state() == icd::TerminalState::DEGRADED || g_aewc.state() == icd::TerminalState::DEGRADED;
}

void term_status_command(icd::TermStatus& ts, LastCommand last) {
    if (last == LastCommand::kTfr) {
        ts.last_cmd_seq = g_tfr.status().last_cmd_seq;
        ts.cmd_rejected = g_tfr.status().cmd_rejected;
    } else if (last == LastCommand::kTargeting) {
        ts.last_cmd_seq = g_targeting.status().last_cmd_seq;
        ts.cmd_rejected = g_targeting.status().cmd_rejected;
    } else if (last == LastCommand::kAewc) {
        ts.last_cmd_seq = g_aewc.status().last_cmd_seq;
        ts.cmd_rejected = g_aewc.status().cmd_rejected;
    } else {
        ts.last_cmd_seq = g_weather.status().last_cmd_seq;
        ts.cmd_rejected = g_weather.status().cmd_rejected;
    }
}

Status load_outputs(bus::RemoteTerminal& terminal, LastCommand last, bus::Word last_sync) {
    Status result = Status::kOk;
    result = fmofp::worst(result, load(terminal, g_weather.status()));
    result = fmofp::worst(result, load(terminal, g_weather.scan_summary()));
    result = fmofp::worst(result, load(terminal, g_tfr.status()));
    result = fmofp::worst(result, load(terminal, g_tfr.guidance()));
    result = fmofp::worst(result, load(terminal, g_targeting.status()));
    result = fmofp::worst(result, load(terminal, g_aewc.status()));

    const bus::BitWord bit = combined_bit();
    icd::TermStatus ts{};
    ts.state = combined_state();
    ts.healthy = g_weather.status().healthy && g_tfr.status().healthy && g_targeting.status().healthy &&
                 g_aewc.status().healthy;
    term_status_command(ts, last);
    ts.bit_in_progress = bit.in_progress;
    ts.bit_failed = bit.failed;
    ts.fault_count = bit.fault_count;
    ts.icd_major = icd::kIcdMajor;
    ts.icd_minor = icd::kIcdMinor;
    ts.last_sync = last_sync;
    result = fmofp::worst(result, load(terminal, ts));
    return result;
}

// Encodes the weather products and publishes them as a new block version.
void publish_weather(const bus::RemoteTerminal& terminal, bus::BlockPublisher& block) {
    std::uint16_t length = 0;
    const Status s =
        g_weather.products().encode(g_wx_payload.data(), static_cast<std::uint16_t>(g_wx_payload.size()), length);
    if (s != Status::kOk && s != Status::kClamped) {
        return;
    }
    if (block.publish(terminal, WxProducts::kSegmentSubaddress, g_wx_payload.data(), length) == Status::kOk) {
        g_weather.set_products_version(block.version());
    }
}

void publish_elevation(const bus::RemoteTerminal& terminal, bus::BlockPublisher& block) {
    std::uint16_t length = 0;
    const Status s =
        g_tfr.products().encode(g_tfr_payload.data(), static_cast<std::uint16_t>(g_tfr_payload.size()), length);
    if (s != Status::kOk && s != Status::kClamped) {
        return;
    }
    (void)block.publish(terminal, TfrProducts::kSegmentSubaddress, g_tfr_payload.data(), length);
}

void publish_tracks(const bus::RemoteTerminal& terminal, bus::BlockPublisher& block) {
    std::uint16_t length = 0;
    const Status s = g_targeting.tracks_block().encode(g_tgt_payload.data(),
                                                      static_cast<std::uint16_t>(g_tgt_payload.size()), length);
    if (s != Status::kOk && s != Status::kClamped) {
        return;
    }
    (void)block.publish(terminal, TgtTracks::kSegmentSubaddress, g_tgt_payload.data(), length);
}

void publish_fused(const bus::RemoteTerminal& terminal, bus::BlockPublisher& block) {
    std::uint16_t length = 0;
    const Status s =
        g_fusion.block().encode(g_fused_payload.data(), static_cast<std::uint16_t>(g_fused_payload.size()), length);
    if (s != Status::kOk && s != Status::kClamped) {
        return;
    }
    (void)block.publish(terminal, FusedTracks::kSegmentSubaddress, g_fused_payload.data(), length);
}

void publish_aewc(const bus::RemoteTerminal& terminal, bus::BlockPublisher& block) {
    std::uint16_t length = 0;
    const Status s =
        g_aewc.tracks_block().encode(g_aewc_payload.data(), static_cast<std::uint16_t>(g_aewc_payload.size()), length);
    if (s != Status::kOk && s != Status::kClamped) {
        return;
    }
    (void)block.publish(terminal, AewcTracks::kSegmentSubaddress, g_aewc_payload.data(), length);
}

Status serve_blocks(bus::RemoteTerminal& terminal, Blocks& blocks) {
    Status result = blocks.weather.serve(terminal, WxProducts::kDescriptorSubaddress, WxProducts::kSegmentSubaddress);
    result = fmofp::worst(
        result, blocks.elevation.serve(terminal, TfrProducts::kDescriptorSubaddress, TfrProducts::kSegmentSubaddress));
    result = fmofp::worst(result,
                          blocks.tracks.serve(terminal, TgtTracks::kDescriptorSubaddress, TgtTracks::kSegmentSubaddress));
    result = fmofp::worst(
        result, blocks.fused.serve(terminal, FusedTracks::kDescriptorSubaddress, FusedTracks::kSegmentSubaddress));
    result = fmofp::worst(result,
                          blocks.aewc.serve(terminal, AewcTracks::kDescriptorSubaddress, AewcTracks::kSegmentSubaddress));
    return result;
}

Status init_blocks(Blocks& blocks) {
    Status result = blocks.weather.init(g_wx_storage.data(), static_cast<std::uint16_t>(g_wx_storage.size()));
    result = fmofp::worst(result,
                          blocks.elevation.init(g_tfr_storage.data(), static_cast<std::uint16_t>(g_tfr_storage.size())));
    result = fmofp::worst(result, blocks.tracks.init(g_tgt_storage.data(), static_cast<std::uint16_t>(g_tgt_storage.size())));
    result = fmofp::worst(result,
                          blocks.fused.init(g_fused_storage.data(), static_cast<std::uint16_t>(g_fused_storage.size())));
    result = fmofp::worst(result, blocks.aewc.init(g_aewc_storage.data(), static_cast<std::uint16_t>(g_aewc_storage.size())));
    result = fmofp::worst(
        result, blocks.terrain.init(g_terrain_storage.data(), static_cast<std::uint16_t>(g_terrain_storage.size())));
    return fmofp::worst(
        result, blocks.traffic.init(g_traffic_storage.data(), static_cast<std::uint16_t>(g_traffic_storage.size())));
}

bool init_logic(Blocks& blocks) {
    return g_weather.init() == Status::kOk && g_tfr.init() == Status::kOk && g_targeting.init() == Status::kOk &&
           g_aewc.init() == Status::kOk && g_fusion.init() == Status::kOk && init_blocks(blocks) == Status::kOk;
}

void publish_all(bus::RemoteTerminal& terminal, Blocks& blocks) {
    if (g_weather.take_products_update()) {
        publish_weather(terminal, blocks.weather);
    }
    if (g_tfr.take_products_update()) {
        publish_elevation(terminal, blocks.elevation);
    }
    if (g_targeting.take_tracks_update()) {
        publish_tracks(terminal, blocks.tracks);
    }
    if (g_aewc.take_tracks_update()) {
        publish_aewc(terminal, blocks.aewc);
    }
    if (g_fusion.take_update()) {
        publish_fused(terminal, blocks.fused);
    }
    (void)serve_blocks(terminal, blocks);
}

void print_frame(bus::Word sync) {
    std::uint16_t major = 0;
    std::uint8_t minor = 0;
    fmofp::bc::decode_sync_word(sync, major, minor);
    const icd::RdrWxScan& scan = g_weather.scan_summary();
    const icd::RdrTfrGuidance& g = g_tfr.guidance();
    std::printf("radar: major frame %u | weather %s %s, scan %u, max %.1f dBZ, %u cells, block v%u | tfr %s %s, "
                "%s, min clearance %.0f ft, %u commands | tgt %s %s, %u tracks, lock %u %s | aewc %s %s, %u tracks "
                "(%u stealth) | fused %u (%u hostile)\n",
                major, icd::to_string(g_weather.state()), icd::to_string(g_weather.mode()), scan.scan_id,
                scan.max_reflectivity, scan.storm_cells, scan.products_version, icd::to_string(g_tfr.state()),
                icd::to_string(g_tfr.mode()), icd::to_string(g.level), g.min_clearance,
                static_cast<unsigned>(g_tfr.products().commands_count), icd::to_string(g_targeting.state()),
                icd::to_string(g_targeting.mode()), static_cast<unsigned>(g_targeting.held_tracks()),
                static_cast<unsigned>(g_targeting.locked_track()), icd::to_string(g_targeting.status().lock_status),
                icd::to_string(g_aewc.state()), icd::to_string(g_aewc.mode()), static_cast<unsigned>(g_aewc.held_tracks()),
                static_cast<unsigned>(g_aewc.stealth_tracks()), static_cast<unsigned>(g_fusion.count()),
                static_cast<unsigned>(g_fusion.hostile_count()));
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
    Blocks blocks;
    if (!init_logic(blocks)) {
        std::fprintf(stderr, "radar: logic init failed\n");
        return 1;
    }
    bus::RemoteTerminal& terminal = process.terminal();
    LastCommand last_command = LastCommand::kNone;
    (void)load_outputs(terminal, last_command, 0);
    (void)serve_blocks(terminal, blocks);
    if (!o.quiet) {
        std::printf("radar: RT %u on 127.0.0.1:%u\n", kRt, static_cast<unsigned>(o.rt.port_base + kRt));
    }

    unsigned long syncs = 0;
    std::uint32_t frame = 0;
    for (;;) {
        (void)process.poll(100 * fmofp::osal::kNsPerMs);
        bus::Word sync = 0;
        if (!terminal.take_sync(sync)) {
            if (terminal.take_reset_request()) {
                g_weather.reset();
                g_tfr.reset();
                g_targeting.reset();
                g_aewc.reset();
                (void)g_fusion.init();
                (void)init_blocks(blocks);
                last_command = LastCommand::kNone;
                (void)load_outputs(terminal, last_command, 0);
                (void)serve_blocks(terminal, blocks);
            }
            continue;
        }
        ++syncs;
        ++frame;
        if (terminal.take_self_test_request()) {
            g_weather.start_self_test();
            g_tfr.start_self_test();
            g_targeting.start_self_test();
            g_aewc.start_self_test();
        }
        terminal.set_bit(combined_bit());

        icd::RdrWxModeCmd wx_cmd{};
        if (take_fresh(terminal, wx_cmd)) {
            g_weather.command_mode(wx_cmd);
            last_command = LastCommand::kWeather;
        }
        icd::RdrTfrModeCmd tfr_cmd{};
        if (take_fresh(terminal, tfr_cmd)) {
            g_tfr.command_mode(tfr_cmd);
            last_command = LastCommand::kTfr;
        }
        icd::RdrTgtModeCmd tgt_cmd{};
        if (take_fresh(terminal, tgt_cmd)) {
            g_targeting.command_mode(tgt_cmd);
            last_command = LastCommand::kTargeting;
        }
        icd::RdrAewcModeCmd aewc_cmd{};
        if (take_fresh(terminal, aewc_cmd)) {
            g_aewc.command_mode(aewc_cmd);
            last_command = LastCommand::kAewc;
        }

        icd::RdrWxWeather weather{};
        const bool have_weather = take_fresh(terminal, weather);
        g_weather.step(have_weather ? &weather : nullptr, frame);

        icd::RdrTfrSensors sensors{};
        const bool have_sensors = take_fresh(terminal, sensors);
        icd::PlantTerrain profile{};
        bool have_profile = false;
        if (blocks.terrain.collect(terminal, Terrain::kDescriptorSubaddress, Terrain::kSegmentSubaddress)) {
            have_profile = Terrain::Layout::decode(blocks.terrain.receiver().payload(),
                                                   blocks.terrain.receiver().length(), profile) == Status::kOk;
        }
        g_tfr.step(have_sensors ? &sensors : nullptr, have_profile ? &profile : nullptr, frame);

        icd::PlantTraffic traffic{};
        bool have_traffic = false;
        if (blocks.traffic.collect(terminal, Traffic::kDescriptorSubaddress, Traffic::kSegmentSubaddress)) {
            have_traffic = Traffic::Layout::decode(blocks.traffic.receiver().payload(),
                                                   blocks.traffic.receiver().length(), traffic) == Status::kOk;
        }
        g_targeting.step(have_traffic ? &traffic : nullptr, frame);
        g_aewc.step(have_traffic ? &traffic : nullptr, frame);
        std::size_t reports = radar::targeting_reports(g_targeting, g_reports.data(), g_reports.size());
        reports += radar::aewc_reports(g_aewc, g_reports.data() + reports, g_reports.size() - reports);
        g_fusion.step(g_reports.data(), reports, g_targeting.operating(), g_aewc.operating(), frame);

        publish_all(terminal, blocks);
        terminal.set_subsystem_flag(any_degraded());
        (void)load_outputs(terminal, last_command, sync);

        if (!o.quiet && syncs % 50 == 1) {
            print_frame(sync);
        }
        if (o.syncs != 0 && syncs >= o.syncs) {
            break;
        }
    }

    const bus::RtCounters& c = terminal.counters();
    std::printf("radar: syncs=%lu commands=%u receives=%u transmits=%u message_errors=%u scans=%u profiles=%u "
                "terrain_blocks=%u traffic_blocks=%u track_updates=%u aewc_updates=%u fusion_epochs=%u state=%s\n",
                syncs, c.commands, c.receives, c.transmits, c.message_errors, g_weather.scans_completed(),
                g_tfr.profiles_assessed(), blocks.terrain.receiver().completed_blocks(),
                blocks.traffic.receiver().completed_blocks(), g_targeting.updates(), g_aewc.updates(),
                static_cast<unsigned>(g_fusion.epochs()), icd::to_string(combined_state()));
    return 0;
}
