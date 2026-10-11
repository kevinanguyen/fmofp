#include <fmofp/nav/navigation.hpp>

#include <cmath>

namespace fmofp::nav {

namespace {

using icd::Ssm;
using icd::TerminalState;

bool usable(Ssm ssm) noexcept { return ssm == Ssm::NORMAL; }

bool commandable(TerminalState state) noexcept {
    switch (state) {
        case TerminalState::STANDBY:
        case TerminalState::NORMAL:
        case TerminalState::TEST:
        case TerminalState::MAINTENANCE:
        case TerminalState::CALIBRATION:
            return true;
        case TerminalState::DEGRADED:
        case TerminalState::EMERGENCY:
        case TerminalState::FAILURE:
        case TerminalState::RECOVERY:
        case TerminalState::INITIALIZING:
            return false;
    }
    return false;
}

constexpr std::uint32_t kSelfTestFrames = 10;
constexpr std::uint8_t kSaturated = 255;

// A fix is believed only near the Earth's surface: the ellipsoid, 60 000 ft
// of altitude and a generous margin. With four coplanar satellites the
// equations also admit a mirror solution far out in space.
constexpr double kMinPlausibleRadiusM = 6.30e6;
constexpr double kMaxPlausibleRadiusM = 6.45e6;

std::uint8_t saturate(std::uint32_t v) noexcept { return v > kSaturated ? kSaturated : static_cast<std::uint8_t>(v); }

// The twelve pseudorange members of PLANT_GPS as an array.
double pseudorange(const icd::PlantGps& g, std::uint8_t satellite) noexcept {
    switch (satellite) {
        case 1:
            return g.pseudorange_1;
        case 2:
            return g.pseudorange_2;
        case 3:
            return g.pseudorange_3;
        case 4:
            return g.pseudorange_4;
        case 5:
            return g.pseudorange_5;
        case 6:
            return g.pseudorange_6;
        case 7:
            return g.pseudorange_7;
        case 8:
            return g.pseudorange_8;
        case 9:
            return g.pseudorange_9;
        case 10:
            return g.pseudorange_10;
        case 11:
            return g.pseudorange_11;
        case 12:
            return g.pseudorange_12;
        default:
            return 0.0;
    }
}

}  // namespace

void NavFusion::init() noexcept {
    // NavDataFusion.__init__: lat 0, lon 0, alt 30 000 ft, covariance 1.
    lat_ = ScalarKalman{0.0, 1.0, kQLat, kGpsRLat};
    lon_ = ScalarKalman{0.0, 1.0, kQLon, kGpsRLon};
    alt_ = ScalarKalman{kInitialAltitudeFt, 1.0, kQAlt, kGpsRAlt};
    fix_count_ = 0;
}

void NavFusion::predict(double dt, double lat_rate_dps, double lon_rate_dps, double alt_rate_fps) noexcept {
    lat_.predict(dt, lat_rate_dps);
    lon_.predict(dt, lon_rate_dps);
    alt_.predict(dt, alt_rate_fps);
}

void NavFusion::update_gps(double lat_deg, double lon_deg, double alt_ft) noexcept {
    lat_.update(lat_deg);
    lon_.update(lon_deg);
    alt_.update(alt_ft);
    ++fix_count_;
}

DeadReckoningRates dead_reckoning_rates(double ground_speed_kt, double heading_deg, double vertical_speed_fpm,
                                        double latitude_deg) noexcept {
    const double heading_rad = heading_deg * kDegToRad;
    const double gs_dps_lat = (ground_speed_kt * 1.68781) / (kMetresPerDegree / kMetresPerFoot);
    const double cos_lat = std::cos(latitude_deg * kDegToRad);
    const double gs_dps_lon = gs_dps_lat / (cos_lat > 0.001 ? cos_lat : 0.001);
    DeadReckoningRates r;
    r.lat_dps = gs_dps_lat * std::cos(heading_rad);
    r.lon_dps = gs_dps_lon * std::sin(heading_rad);
    r.alt_fps = vertical_speed_fpm / 60.0;
    return r;
}

Status Navigation::init() noexcept {
    constellation_ = build_constellation();
    fusion_.init();
    solution_ = icd::NavSolution{};
    last_sensors_ = icd::PlantSensors{};
    have_sensors_ = false;
    last_sensor_frame_ = 0;
    have_gps_ = false;
    last_epoch_ = 0;
    have_fix_ = false;
    last_fix_ = Fix{};
    gps_valid_ = false;
    sats_used_ = 0;
    frames_since_fix_ = 0;
    state_ = TerminalState::INITIALIZING;
    last_cmd_seq_ = 0;
    cmd_rejected_ = false;
    cmd_seq_ = bus::CommandSequence{};
    bit_ = bus::BitWord{};
    self_test_running_ = false;
    self_test_started_ = 0;
    produce_solution();
    return Status::kOk;
}

void Navigation::reset() noexcept { (void)init(); }

void Navigation::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void Navigation::command(const icd::TermCmd& cmd) noexcept {
    if (!cmd_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    last_cmd_seq_ = cmd.cmd_seq;
    if (!commandable(cmd.requested_state)) {
        cmd_rejected_ = true;
        return;
    }
    cmd_rejected_ = false;
    state_ = cmd.requested_state;
}

void Navigation::step(const icd::NavSensors* sensors, const icd::NavGps* gps, std::uint32_t frame) noexcept {
    if (sensors != nullptr) {
        last_sensors_ = sensors->payload;
        have_sensors_ = true;
        last_sensor_frame_ = frame;
    }
    const bool fresh = have_sensors_ && frame - last_sensor_frame_ < kSensorStaleFrames;

    if (self_test_running_) {
        if (self_test_started_ == 0) {
            self_test_started_ = frame == 0 ? 1 : frame;
        } else if (frame - self_test_started_ >= kSelfTestFrames) {
            self_test_running_ = false;
            self_test_started_ = 0;
            bit_.in_progress = false;
        }
    }

    // INITIALIZING until the first sensors; NORMAL while they keep coming;
    // DEGRADED when they stop. A commanded state is kept until commanded again.
    if (state_ == TerminalState::INITIALIZING && fresh) {
        state_ = TerminalState::NORMAL;
    } else if (state_ == TerminalState::NORMAL && !fresh) {
        state_ = TerminalState::DEGRADED;
    } else if (state_ == TerminalState::DEGRADED && fresh) {
        state_ = TerminalState::NORMAL;
    }

    // Dead reckoning every frame (the Python predicted at the FMS rate with
    // the wall-clock dt). Without usable sensors the position is held.
    const icd::PlantSensors& s = last_sensors_;
    DeadReckoningRates rates{};
    if (fresh && usable(s.airspeed_ssm) && usable(s.heading_ssm) && usable(s.vertical_speed_ssm)) {
        rates = dead_reckoning_rates(s.airspeed, s.heading, s.vertical_speed, fusion_.latitude());
    }
    fusion_.predict(kFrameSeconds, rates.lat_dps, rates.lon_dps, rates.alt_fps);

    // A GPS epoch is solved once, when it first arrives.
    if (gps != nullptr && (!have_gps_ || gps->payload.epoch != last_epoch_)) {
        have_gps_ = true;
        last_epoch_ = gps->payload.epoch;
        solve_epoch(gps->payload);
    }
    if (have_fix_ && frames_since_fix_ < 0xFFFFFFFFU) {
        ++frames_since_fix_;
    }
    produce_solution();
}

void Navigation::solve_epoch(const icd::PlantGps& gps) noexcept {
    std::array<Observation, kSatellites> obs{};
    std::size_t n = 0;
    for (std::uint8_t sat = 1; sat <= kSatellites; ++sat) {
        if ((static_cast<unsigned>(gps.visible_mask) & (1U << (sat - 1U))) != 0U) {
            obs[n].satellite = sat;
            obs[n].pseudorange_m = pseudorange(gps, sat);
            ++n;
        }
    }
    // The Python iterated from its last fix, or from the simulation's truth
    // before there was one; here the first start is a closed-form acquisition.
    Fix start = last_fix_;
    if (!have_fix_ && bancroft(constellation_, obs.data(), n, start) != Status::kOk) {
        gps_valid_ = false;
        return;
    }
    Fix fix;
    const Status s = wls_solve(constellation_, obs.data(), n, start.position, start.clock_bias_s, fix);
    const double radius = std::sqrt(fix.position.x * fix.position.x + fix.position.y * fix.position.y +
                                    fix.position.z * fix.position.z);
    if (s != Status::kOk || !fix.converged || radius < kMinPlausibleRadiusM || radius > kMaxPlausibleRadiusM) {
        gps_valid_ = false;  // GPSSystem._fix_valid False: the last position stands
        return;
    }
    last_fix_ = fix;
    have_fix_ = true;
    gps_valid_ = true;
    sats_used_ = static_cast<std::uint8_t>(n);
    frames_since_fix_ = 0;
    const Geodetic g = ecef_to_wgs84(fix.position);
    fusion_.update_gps(g.latitude_deg, g.longitude_deg, g.altitude_ft);
}

void Navigation::produce_solution() noexcept {
    const bool have_position = fusion_.fix_count() > 0;
    solution_.position_ssm = have_position ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    solution_.altitude_ssm = have_position ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    solution_.gps_valid = gps_valid_;
    solution_.sats_used = sats_used_;
    solution_.latitude = fusion_.latitude();
    solution_.longitude = fusion_.longitude();
    solution_.altitude = fusion_.altitude_ft();
    solution_.fix_count = saturate(fusion_.fix_count());
    solution_.fix_age = have_fix_ ? saturate(frames_since_fix_ / kFramesPerSecond) : kSaturated;
    solution_.clock_bias = have_fix_ ? last_fix_.clock_bias_s * 1e6 : 0.0;
}

}  // namespace fmofp::nav
