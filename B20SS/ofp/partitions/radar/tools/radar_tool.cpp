// fmofp_radar_tool: the radar computer's pure functions on stdin/stdout, so the
// Python tests can run the original Python beside them
// (B20SS/fmofp_bus/tests/test_weather_oracle.py, test_tfr_oracle.py). Test
// support, not flight code.
//
// One command per line; numbers are printed with 17 significant digits. The
// tool keeps a weather truth, a mode, a texture, a range and one WeatherRadar,
// plus a terrain profile and the clearance manager's latch.
//   sigfigs <x> [n]                               -> round_to_sigfigs
//   truth <n> {az range size dbz profile}*n <m> [az range width depth dbz]   (m is 0 or 1)
//   mode SURVEILLANCE|MAPPING|TURBULENCE|WINDSHEAR|PRECIPITATION
//   texture <stratiform_dbz> <clutter_dbz> [seed]   (0 0 turns the texture off)
//   range <km>
//   thickness                                     -> the mode's layer thicknesses
//   volume <azimuths> <ranges>                    -> a reduced volume: "az el v0 ... v(ranges-1)" lines, then "end"
//   scan [frame]                                  -> runs a whole scan; prints "scan <id> <max_dbz>"
//   field vil|precip|composite                    -> 360 lines of 200 values, then "end"
//   points vil|precip                             -> "n <count> <significant> <max>", then one line per point, "end"
//   cells | areas | windshear | turbulence        -> one line per item, then "end"
//   summary                                       -> the RDR_WX_SCAN numbers
//   payload                                       -> "payload <length> <hex words...>" of the products block
// Terrain following (tfr_processor.py, tfr_radar.py):
//   tfr_profile <range_m> <n> <elevation_m>*n     -> ok; the profile the next commands work on
//   tfr_assess <altitude_m>                       -> "<master> <min_clearance_m> <min_index> <advisories>",
//                                                    then "i distance level" per sample, "end"; updates the latch
//   tfr_reset                                     -> the latch back to CLEAR
//   tfr_path <altitude_m> <speed_mps> <clearance_m> -> "distance altitude reason" per command, "end"
//   tfr_warnings                                  -> "i distance warning" per sample, "end"
// AEWC (aewc_radar.py, aewc_processor.py):
//   aewc_snr <rcs> <range_m>                      -> the radar equation's SNR in dB (0 when negative)
//   aewc_sector <x> <y> <z>                       -> the sector 1-6 of a position, 0 outside the elevation band
//   aewc_class <speed> <z> <stealth 0|1>          -> the classification
//   aewc_ecm <exceedance_db>                      -> "<jammed> <severity> <jam_type> <mitigation>"
//   aewc_priority <6 counts> <6 hostile> <6 ages_s> <jammed mask>
//                                                 -> "order s1..s6 dwell d1..d6 score c1..c6"
// Targeting (target_processor.py) and fusion (radar_data_fusion.py):
//   signature <rcs> <scintillation> <speed> <altitude> -> "<class> <confidence> <rcs_corrected>"
//   stealth_reset                                 -> the detector's state for one track back to the prior
//   stealth <rcs> <snr>                           -> "<posterior> <snr_integrated> <pulses> <is_stealth>"
//   threat <class> <identity> <avg_rcs> <speed>   -> "<threat>"
//   fusion_reset                                  -> a fresh fusion (no previous epoch)
//   fusion <n>, then n lines "x y z vx vy vz class identity rcs weight sensor"
//                                                 -> "n <count> <hostile>", then "id x y z vx vy vz confidence
//                                                    class identity threat sources" per track, "end"

#include <array>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/radar/target_processing.hpp>
#include <fmofp/radar/terrain_following.hpp>
#include <fmofp/radar/aewc_radar.hpp>
#include <fmofp/radar/track_fusion.hpp>
#include <fmofp/radar/weather_model.hpp>
#include <fmofp/radar/weather_products.hpp>
#include <fmofp/radar/weather_radar.hpp>

namespace {

namespace radar = fmofp::radar;
namespace icd = fmofp::icd;

radar::WeatherRadar g_radar;
radar::Volume g_volume;
radar::WeatherTruth g_truth;
icd::WeatherMode g_mode = icd::WeatherMode::SURVEILLANCE;
radar::Texture g_texture;
double g_range_km = 200.0;
std::uint32_t g_frame = 0;
std::vector<fmofp::bus::Word> g_payload(icd::RdrWxProducts::kMaxPayloadWords);
radar::TerrainProfile g_profile;
icd::ClearanceLevel g_master = icd::ClearanceLevel::CLEAR;
radar::StealthState g_stealth;
radar::TrackFusion g_fusion;
std::vector<radar::TrackReport> g_reports;

// An enumerator by its ICD name, for the enums the tool reads.
template <typename Enum>
bool parse_enum(const std::string& name, Enum& out) {
    for (unsigned v = 0; v < 16U; ++v) {
        const Enum candidate = static_cast<Enum>(v);
        if (icd::is_valid(candidate) && name == icd::to_string(candidate)) {
            out = candidate;
            return true;
        }
    }
    return false;
}

void do_fusion(std::size_t n) {
    g_reports.assign(n, radar::TrackReport{});
    std::string line;
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::getline(std::cin, line)) {
            std::printf("error kInvalidArgument\n");
            return;
        }
        std::istringstream in(line);
        std::string cls;
        std::string identity;
        double sensor = 0.0;
        radar::TrackReport& r = g_reports[i];
        if (!(in >> r.position[0] >> r.position[1] >> r.position[2] >> r.velocity[0] >> r.velocity[1] >> r.velocity[2] >>
              cls >> identity >> r.rcs >> r.weight >> sensor) ||
            !parse_enum(cls, r.classification) || !parse_enum(identity, r.identity)) {
            std::printf("error kInvalidArgument\n");
            return;
        }
        r.sensor = static_cast<std::uint8_t>(sensor);
    }
    g_fusion.epoch(g_reports.data(), n, true, false);
    std::printf("n %u %u\n", static_cast<unsigned>(g_fusion.count()), static_cast<unsigned>(g_fusion.hostile_count()));
    for (std::size_t i = 0; i < g_fusion.count(); ++i) {
        const radar::FusedTrack& f = g_fusion.track(i);
        std::printf("%u %.17g %.17g %.17g %.17g %.17g %.17g %.17g %s %s %s %u\n", static_cast<unsigned>(f.id),
                    f.position[0], f.position[1], f.position[2], f.velocity[0], f.velocity[1], f.velocity[2],
                    f.confidence, icd::to_string(f.classification), icd::to_string(f.identity),
                    icd::to_string(f.threat), static_cast<unsigned>(f.sources));
    }
    std::printf("end\n");
}

void print_values(const std::vector<double>& values) {
    bool first = true;
    for (const double v : values) {
        std::printf(first ? "%.17g" : " %.17g", v);
        first = false;
    }
    std::printf("\n");
}

bool read(std::istringstream& in, std::vector<double>& out, std::size_t count) {
    out.clear();
    double value = 0.0;
    while (out.size() < count && in >> value) {
        out.push_back(value);
    }
    return out.size() == count;
}

bool parse_mode(const std::string& name, icd::WeatherMode& mode) {
    for (const icd::WeatherMode m : {icd::WeatherMode::SURVEILLANCE, icd::WeatherMode::MAPPING,
                                     icd::WeatherMode::TURBULENCE, icd::WeatherMode::WINDSHEAR,
                                     icd::WeatherMode::PRECIPITATION, icd::WeatherMode::NONE}) {
        if (name == icd::to_string(m)) {
            mode = m;
            return true;
        }
    }
    return false;
}

void print_field(const radar::Field& f) {
    for (const auto& row : f) {
        print_values(std::vector<double>(row.begin(), row.end()));
    }
    std::printf("end\n");
}

void print_points(const radar::PointProducts& p) {
    std::printf("n %zu %u %.17g\n", p.count, p.significant, p.max_value);
    for (std::size_t i = 0; i < p.count; ++i) {
        const radar::Point& q = p.points[i];
        std::printf("%zu %zu %.17g %.17g %u %d %s %.17g %.17g\n", q.az_index, q.r_index, q.value, q.intensity,
                    static_cast<unsigned>(q.layer_count), q.show_values ? 1 : 0, icd::to_string(q.type), q.x_nm, q.y_nm);
    }
    std::printf("end\n");
}

void do_volume(std::size_t azimuths, std::size_t ranges) {
    radar::Geometry g;
    g.azimuths = azimuths;
    g.ranges = ranges;
    g.range_bins = ranges;
    const radar::Vcp vcp = radar::vcp_for(g_mode);
    for (std::size_t az = 0; az < azimuths; ++az) {
        radar::render_azimuth(g_truth, vcp, g, g_texture, az, g_volume);
        for (std::size_t el = 0; el < vcp.count; ++el) {
            std::printf("%zu %zu", az, el);
            for (std::size_t r = 0; r < ranges; ++r) {
                std::printf(" %.17g", g_volume.dbz[az][el][r]);
            }
            std::printf("\n");
        }
    }
    std::printf("end\n");
}

void do_scan(std::uint32_t frame) {
    g_frame = frame;
    g_radar.set_texture(g_texture);
    g_radar.set_range_km(g_range_km);
    g_radar.scan_now(g_truth, g_mode, frame);
    std::printf("scan %u %.17g\n", static_cast<unsigned>(g_radar.scan_summary().scan_id), g_radar.max_reflectivity());
}

void do_summary() {
    const icd::RdrWxScan& s = g_radar.scan_summary();
    std::printf("%u %s %.17g %u %u %u %u %u %u %.17g %d %s %s\n", static_cast<unsigned>(s.scan_id), icd::to_string(s.mode),
                s.max_reflectivity, static_cast<unsigned>(s.storm_cells), static_cast<unsigned>(s.precip_areas),
                static_cast<unsigned>(s.windshear_events), static_cast<unsigned>(s.turbulence_cells),
                static_cast<unsigned>(s.vil_points), static_cast<unsigned>(s.precip_points), s.vil_max,
                s.microburst ? 1 : 0, icd::to_string(s.max_turbulence), icd::to_string(s.max_shear));
}

void do_payload() {
    std::uint16_t length = 0;
    const fmofp::Status s =
        g_radar.products().encode(g_payload.data(), static_cast<std::uint16_t>(g_payload.size()), length);
    if (s != fmofp::Status::kOk && s != fmofp::Status::kClamped) {
        std::printf("error %s\n", fmofp::to_string(s));
        return;
    }
    std::printf("payload %u", static_cast<unsigned>(length));
    for (std::uint16_t i = 0; i < length; ++i) {
        std::printf(" %04X", static_cast<unsigned>(g_payload[i]));
    }
    std::printf("\n");
}

bool do_tfr_profile(std::istringstream& in) {
    std::vector<double> head;
    if (!read(in, head, 2)) {
        return false;
    }
    const auto n = static_cast<std::size_t>(head[1]);
    std::vector<double> heights;
    if (n == 0 || n > radar::kMaxProfileSamples || !read(in, heights, n)) {
        return false;
    }
    g_profile = radar::TerrainProfile{};
    g_profile.range_m = head[0];
    g_profile.samples = n;
    for (std::size_t i = 0; i < n; ++i) {
        g_profile.elevation_m[i] = heights[i];
    }
    return true;
}

void do_tfr_assess(double altitude_m) {
    radar::ClearanceAssessment a{};
    radar::assess_profile(g_profile, altitude_m, g_master, a);
    g_master = a.master;
    std::printf("%s %.17g %zu %zu\n", icd::to_string(a.master), a.min_clearance_m, a.min_index, a.advisories);
    for (std::size_t i = 0; i < g_profile.samples; ++i) {
        std::printf("%zu %.17g %s\n", i, g_profile.distance_m(i), icd::to_string(a.level[i]));
    }
    std::printf("end\n");
}

void do_tfr_path(double altitude_m, double speed_mps, double clearance_m) {
    radar::PathCommands c{};
    radar::compute_path(g_profile, altitude_m, speed_mps, clearance_m, c);
    for (std::size_t i = 0; i < c.count; ++i) {
        std::printf("%.17g %.17g %s\n", c.command[i].distance_m, c.command[i].altitude_m,
                    icd::to_string(c.command[i].reason));
    }
    std::printf("end\n");
}

void do_tfr_warnings() {
    for (std::size_t i = 0; i < g_profile.samples; ++i) {
        std::printf("%zu %.17g %s\n", i, g_profile.distance_m(i), icd::to_string(radar::terrain_warning(g_profile.elevation_m[i])));
    }
    std::printf("end\n");
}

}  // namespace

int main() {
    (void)g_radar.init();
    (void)g_fusion.init();
    std::string line;
    std::vector<double> a;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd) || cmd.empty() || cmd[0] == '#') {
            continue;
        }
        if (cmd == "signature" && read(in, a, 4)) {
            const radar::SignatureResult r = radar::analyse_signature(a[0], a[1], a[2], a[3]);
            std::printf("%s %.17g %.17g\n", icd::to_string(r.type), r.confidence, r.rcs_corrected);
        } else if (cmd == "stealth_reset") {
            g_stealth = radar::StealthState{};
            std::printf("ok\n");
        } else if (cmd == "stealth" && read(in, a, 2)) {
            const radar::StealthResult r = radar::assess_stealth(g_stealth, a[0], a[1]);
            std::printf("%.17g %.17g %u %d\n", r.posterior, r.snr_integrated_db, static_cast<unsigned>(r.pulses),
                        r.is_stealth ? 1 : 0);
        } else if (cmd == "threat") {
            std::string cls;
            std::string identity;
            icd::TargetClass type = icd::TargetClass::UNKNOWN;
            icd::IffIdentity iff = icd::IffIdentity::UNKNOWN;
            if (!(in >> cls >> identity) || !parse_enum(cls, type) || !parse_enum(identity, iff) || !read(in, a, 2)) {
                std::printf("error kInvalidArgument\n");
                continue;
            }
            std::printf("%s\n", icd::to_string(radar::assess_threat(type, iff, a[0], a[1])));
        } else if (cmd == "fusion_reset") {
            (void)g_fusion.init();
            std::printf("ok\n");
        } else if (cmd == "fusion" && read(in, a, 1)) {
            do_fusion(static_cast<std::size_t>(a[0]));
        } else if (cmd == "aewc_snr" && read(in, a, 2)) {
            std::printf("%.17g\n", radar::aewc_snr_db(a[0], a[1]));
        } else if (cmd == "aewc_sector" && read(in, a, 3)) {
            const std::array<double, 3> position{a[0], a[1], a[2]};
            std::printf("%u\n", static_cast<unsigned>(radar::aewc_sector_of(position)));
        } else if (cmd == "aewc_class" && read(in, a, 3)) {
            std::printf("%s\n", icd::to_string(radar::aewc_classify(a[0], a[1], a[2] != 0.0)));
        } else if (cmd == "aewc_ecm" && read(in, a, 1)) {
            const radar::EcmAssessment e = radar::ecm_assess(a[0]);
            std::printf("%d %.17g %s %s\n", e.jammed ? 1 : 0, e.severity, icd::to_string(e.jam_type),
                        icd::to_string(e.mitigation));
        } else if (cmd == "aewc_priority" && read(in, a, 19)) {
            std::array<std::uint8_t, radar::kAewcSectorCount> tracks{};
            std::array<std::uint8_t, radar::kAewcSectorCount> hostile{};
            std::array<double, radar::kAewcSectorCount> ages{};
            for (std::size_t i = 0; i < radar::kAewcSectorCount; ++i) {
                tracks[i] = static_cast<std::uint8_t>(a[i]);
                hostile[i] = static_cast<std::uint8_t>(a[radar::kAewcSectorCount + i]);
                ages[i] = a[2 * radar::kAewcSectorCount + i];
            }
            const radar::SectorPriorities p =
                radar::aewc_priorities(tracks, hostile, ages, static_cast<std::uint8_t>(a[3 * radar::kAewcSectorCount]));
            std::printf("order");
            for (std::uint8_t index : p.order) {
                std::printf(" %u", static_cast<unsigned>(index) + 1U);
            }
            std::printf(" dwell");
            for (double d : p.dwell) {
                std::printf(" %.17g", d);
            }
            std::printf(" score");
            for (double s : p.score) {
                std::printf(" %.17g", s);
            }
            std::printf("\n");
        } else if (cmd == "sigfigs" && read(in, a, 1)) {
            double n = 3.0;
            in >> n;
            print_values({radar::round_to_sigfigs(a[0], static_cast<int>(n))});
        } else if (cmd == "truth" && read(in, a, 1)) {
            const auto n = static_cast<std::size_t>(a[0]);
            std::vector<double> cells;
            std::vector<double> m;
            if (n > radar::kMaxTruthCells || !read(in, cells, 5 * n) || !read(in, m, 1)) {
                std::printf("error kInvalidArgument\n");
                continue;
            }
            g_truth = radar::WeatherTruth{};
            g_truth.cell_count = n;
            for (std::size_t i = 0; i < n; ++i) {
                g_truth.cells[i] = radar::TruthCell{cells[5 * i], cells[5 * i + 1], cells[5 * i + 2], cells[5 * i + 3],
                                                    cells[5 * i + 4] != 0.0};
            }
            if (m[0] != 0.0) {
                std::vector<double> area;
                if (!read(in, area, 5)) {
                    std::printf("error kInvalidArgument\n");
                    continue;
                }
                g_truth.has_area = true;
                g_truth.area = radar::TruthArea{area[0], area[1], area[2], area[3], area[4]};
            }
            std::printf("ok\n");
        } else if (cmd == "mode") {
            std::string name;
            in >> name;
            std::printf(parse_mode(name, g_mode) ? "ok\n" : "error kInvalidArgument\n");
        } else if (cmd == "texture" && read(in, a, 2)) {
            g_texture.stratiform_dbz = a[0];
            g_texture.clutter_dbz = a[1];
            double seed = 0.0;
            if (in >> seed) {
                g_texture.seed = static_cast<std::uint32_t>(seed);
            }
            std::printf("ok\n");
        } else if (cmd == "range" && read(in, a, 1)) {
            g_range_km = a[0];
            std::printf("ok\n");
        } else if (cmd == "thickness") {
            const auto t = radar::layer_thicknesses(radar::vcp_for(g_mode));
            print_values(std::vector<double>(t.begin(), t.begin() + static_cast<std::ptrdiff_t>(radar::vcp_for(g_mode).count)));
        } else if (cmd == "volume" && read(in, a, 2)) {
            const auto azimuths = static_cast<std::size_t>(a[0]);
            const auto ranges = static_cast<std::size_t>(a[1]);
            if (azimuths == 0 || azimuths > radar::kAzimuths || ranges == 0 || ranges > radar::kRanges) {
                std::printf("error kInvalidArgument\n");
                continue;
            }
            do_volume(azimuths, ranges);
        } else if (cmd == "scan") {
            double frame = 0.0;
            const bool given = static_cast<bool>(in >> frame);
            do_scan(given ? static_cast<std::uint32_t>(frame) : g_frame + radar::kFramesPerScan);
        } else if (cmd == "field") {
            std::string which;
            in >> which;
            if (which == "vil") {
                print_field(g_radar.vil_field());
            } else if (which == "precip") {
                print_field(g_radar.precip_field());
            } else if (which == "composite") {
                print_field(g_radar.composite_field());
            } else {
                std::printf("error kInvalidArgument\n");
            }
        } else if (cmd == "points") {
            std::string which;
            in >> which;
            if (which == "vil") {
                print_points(g_radar.vil_points());
            } else if (which == "precip") {
                print_points(g_radar.precip_points());
            } else {
                std::printf("error kInvalidArgument\n");
            }
        } else if (cmd == "cells") {
            const radar::StormCellTracker& t = g_radar.tracker();
            for (std::size_t i = 0; i < t.size(); ++i) {
                const radar::TrackedCell& c = t.cell_at(i);
                std::printf("%u %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", static_cast<unsigned>(c.id),
                            c.center_az, c.center_r, c.reflectivity, c.size, c.intensity, c.velocity_az, c.velocity_r,
                            c.altitude, c.vertical_development);
            }
            std::printf("end\n");
        } else if (cmd == "areas") {
            const radar::PrecipitationAnalyzer& an = g_radar.analyzer();
            for (std::size_t i = 0; i < an.size(); ++i) {
                const radar::PrecipArea& p = an.area_at(i);
                std::printf("%.17g %.17g %.17g %.17g %s %.17g\n", p.center_az, p.center_r, p.reflectivity, p.rate,
                            icd::to_string(p.type), p.intensity);
            }
            std::printf("end\n");
        } else if (cmd == "windshear") {
            const radar::WindShearProcessor& w = g_radar.windshear();
            for (std::size_t i = 0; i < w.size(); ++i) {
                const radar::ShearEvent& e = w.event_at(i);
                std::printf("%zu %zu %.17g %.17g %d %s %.17g\n", e.az, e.r, e.shear_kts, e.divergence,
                            e.microburst ? 1 : 0, icd::to_string(e.severity), e.altitude_ft);
            }
            std::printf("end\n");
        } else if (cmd == "turbulence") {
            const radar::TurbulenceProcessor& t = g_radar.turbulence();
            for (std::size_t i = 0; i < t.size(); ++i) {
                const radar::TurbulenceCell& c = t.cell_at(i);
                std::printf("%zu %zu %.17g %.17g %s %.17g %.17g\n", c.az, c.r, c.intensity, c.spectrum_width,
                            icd::to_string(c.category), c.altitude_ft, c.edr);
            }
            std::printf("end\n");
        } else if (cmd == "summary") {
            do_summary();
        } else if (cmd == "payload") {
            do_payload();
        } else if (cmd == "tfr_profile") {
            std::printf(do_tfr_profile(in) ? "ok\n" : "error kInvalidArgument\n");
        } else if (cmd == "tfr_assess" && read(in, a, 1)) {
            do_tfr_assess(a[0]);
        } else if (cmd == "tfr_reset") {
            g_master = icd::ClearanceLevel::CLEAR;
            std::printf("ok\n");
        } else if (cmd == "tfr_path" && read(in, a, 3)) {
            do_tfr_path(a[0], a[1], a[2]);
        } else if (cmd == "tfr_warnings") {
            do_tfr_warnings();
        } else {
            std::printf("error unknown command '%s'\n", cmd.c_str());
        }
        std::fflush(stdout);
    }
    return 0;
}
