#include <fmofp/radar/terrain_following.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::radar {

namespace {

using icd::ClearanceLevel;
using icd::PathReason;
using icd::Ssm;
using icd::TerminalState;
using icd::TerrainWarning;
using icd::TfrMode;

// Self test takes a fixed number of frames so TERM_STATUS shows it in progress.
constexpr std::uint32_t kSelfTestFrames = 10;

bool usable(Ssm ssm) noexcept { return ssm == Ssm::NORMAL; }

// A state the radar can be commanded into from the bus.
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

// ClearanceManager's order {CLEAR 0, LOW 1, CAUTION 2, PULL_UP 3}; NONE ranks lowest.
unsigned rank(ClearanceLevel level) noexcept {
    switch (level) {
        case ClearanceLevel::NONE:
        case ClearanceLevel::CLEAR:
            return 0U;
        case ClearanceLevel::LOW:
            return 1U;
        case ClearanceLevel::CAUTION:
            return 2U;
        case ClearanceLevel::PULL_UP:
            return 3U;
    }
    return 0U;
}

std::uint8_t saturate8(std::size_t n) noexcept { return static_cast<std::uint8_t>(std::min<std::size_t>(n, 255)); }

}  // namespace

double TerrainProfile::distance_m(std::size_t i) const noexcept {
    if (samples < 2U) {
        return 0.0;
    }
    if (i + 1U >= samples) {
        return range_m;
    }
    return static_cast<double>(i) * (range_m / static_cast<double>(samples - 1U));
}

ClearanceLevel clearance_level(double clearance_m) noexcept {
    if (clearance_m >= kClearM) {
        return ClearanceLevel::CLEAR;
    }
    if (clearance_m >= kLowM) {
        return ClearanceLevel::LOW;
    }
    if (clearance_m >= kCautionM) {
        return ClearanceLevel::CAUTION;
    }
    return ClearanceLevel::PULL_UP;
}

void assess_profile(const TerrainProfile& profile, double altitude_m, ClearanceLevel previous_master,
                    ClearanceAssessment& out) noexcept {
    out = ClearanceAssessment{};
    const std::size_t n = std::min(profile.samples, kMaxProfileSamples);
    ClearanceLevel worst_level = ClearanceLevel::CLEAR;
    bool first = true;
    for (std::size_t i = 0; i < n; ++i) {
        const double clearance = altitude_m - profile.elevation_m[i];
        if (first || clearance < out.min_clearance_m) {
            out.min_clearance_m = clearance;
            out.min_index = i;
            first = false;
        }
        const ClearanceLevel level = clearance_level(clearance);
        out.level[i] = level;
        if (level != ClearanceLevel::CLEAR) {
            ++out.advisories;
            if (rank(level) > rank(worst_level)) {
                worst_level = level;
            }
        }
    }
    // The master level only rises while advisories exist, and resets when none do.
    const ClearanceLevel held = previous_master == ClearanceLevel::NONE ? ClearanceLevel::CLEAR : previous_master;
    if (out.advisories == 0) {
        out.master = ClearanceLevel::CLEAR;
    } else {
        out.master = rank(worst_level) > rank(held) ? worst_level : held;
    }
}

void compute_path(const TerrainProfile& profile, double current_alt_m, double speed_mps, double clearance_m,
                  PathCommands& out) noexcept {
    out = PathCommands{};
    const std::size_t n = std::min(profile.samples, kMaxProfileSamples);
    if (n == 0) {
        return;
    }
    // The window: every sample within the look-ahead, else the first twenty.
    std::size_t window = 0;
    while (window < n && profile.distance_m(window) <= kLookaheadM) {
        ++window;
    }
    if (window == 0) {
        window = std::min(n, kFallbackWindowSamples);
    }
    const std::size_t window_size = std::max(kMinWindowSamples, window / 10U);
    const double speed = std::max(speed_mps, 1.0);

    double prev_alt = current_alt_m;
    double max_in_window = profile.elevation_m[0];
    for (std::size_t i = 0; i < window; ++i) {
        max_in_window = std::max(max_in_window, profile.elevation_m[i]);
        const std::size_t ahead_end = std::min(window, i + window_size);
        double max_terrain = profile.elevation_m[i];
        for (std::size_t j = i + 1U; j < ahead_end; ++j) {
            max_terrain = std::max(max_terrain, profile.elevation_m[j]);
        }
        const double required_alt = max_terrain + clearance_m;
        const double dist = profile.distance_m(i);
        const double max_delta = kMaxAltRateMps * (dist / speed);
        const double ceiling = prev_alt + max_delta < required_alt ? prev_alt + max_delta : required_alt;
        const double commanded = prev_alt - max_delta > ceiling ? prev_alt - max_delta : ceiling;
        if (out.count == 0 || std::fabs(commanded - out.command[out.count - 1U].altitude_m) > kCommandStepM) {
            if (out.count < kMaxPathCommands) {
                out.command[out.count] = PathCommand{dist, commanded, PathReason::TERRAIN_AVOIDANCE};
                ++out.count;
            }
            prev_alt = commanded;  // the Python moved its reference only when it emitted a command
        }
    }
    const double clearance_alt = max_in_window + clearance_m;
    if ((out.count == 0 || out.command[out.count - 1U].altitude_m < clearance_alt) && out.count < kMaxPathCommands) {
        out.command[out.count] = PathCommand{profile.distance_m(window - 1U), clearance_alt, PathReason::CLEARANCE};
        ++out.count;
    }
}

TerrainWarning terrain_warning(double elevation_m) noexcept {
    if (elevation_m > kHighTerrainM) {
        return TerrainWarning::HIGH_TERRAIN;
    }
    if (std::fabs(elevation_m - kSteepReferenceM) > kSteepBandM) {
        return TerrainWarning::STEEP_TERRAIN;
    }
    return TerrainWarning::NONE;
}

Status TerrainFollowing::init() noexcept {
    status_ = icd::RdrTfrStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    guidance_ = icd::RdrTfrGuidance{};
    invalidate_guidance();
    products_ = icd::RdrTfrElevation{};
    products_dirty_ = true;  // the empty block is published first
    products_cleared_ = true;
    last_block_frame_ = 0;
    block_published_ = false;
    mode_ = TfrMode::NONE;
    set_clearance_ft_ = kDefaultSetClearanceFt;
    mode_seq_ = bus::CommandSequence{};
    sensors_ = icd::PlantSensors{};
    have_sensors_ = false;
    sensor_frame_ = 0;
    profile_ = TerrainProfile{};
    have_profile_ = false;
    profile_unassessed_ = false;
    profile_frame_ = 0;
    master_ = ClearanceLevel::CLEAR;
    assessment_ = ClearanceAssessment{};
    path_ = PathCommands{};
    profiles_assessed_ = 0;
    mode_transition_ = false;
    self_test_running_ = false;
    self_test_started_ = 0;
    bit_ = bus::BitWord{};
    return Status::kOk;
}

void TerrainFollowing::reset() noexcept { (void)init(); }

void TerrainFollowing::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void TerrainFollowing::run_self_test(std::uint32_t frame) noexcept {
    if (!self_test_running_) {
        return;
    }
    if (self_test_started_ == 0) {
        self_test_started_ = frame == 0 ? 1 : frame;
    } else if (frame - self_test_started_ >= kSelfTestFrames) {
        self_test_running_ = false;
        self_test_started_ = 0;
        bit_.in_progress = false;  // nothing in this partition can fail yet
    }
}

bool TerrainFollowing::operating() const noexcept {
    return status_.state == TerminalState::NORMAL && mode_ != TfrMode::NONE;
}

void TerrainFollowing::command_mode(const icd::RdrTfrModeCmd& cmd) noexcept {
    if (!mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.last_cmd_seq = cmd.cmd_seq;
    if (!commandable(cmd.requested_state) || !icd::is_valid(cmd.requested_mode)) {
        status_.cmd_rejected = true;
        return;
    }
    status_.cmd_rejected = false;
    status_.state = cmd.requested_state;
    const TfrMode requested = cmd.requested_state == TerminalState::NORMAL ? cmd.requested_mode : TfrMode::NONE;
    mode_transition_ = requested != mode_ && requested != TfrMode::NONE;
    mode_ = requested;
    if (std::isfinite(cmd.set_clearance)) {
        set_clearance_ft_ = std::fmin(kMaxSetClearanceFt, std::fmax(kMinSetClearanceFt, cmd.set_clearance));
    }
    profile_unassessed_ = have_profile_;  // the next assessment republishes the block for the new settings
}

void TerrainFollowing::step(const icd::RdrTfrSensors* sensors, const icd::PlantTerrain* terrain,
                            std::uint32_t frame) noexcept {
    if (sensors != nullptr) {
        sensors_ = sensors->payload;
        have_sensors_ = true;
        sensor_frame_ = frame;
    }
    if (terrain != nullptr && terrain->elevation_count != 0U) {
        profile_.samples = std::min<std::size_t>(terrain->elevation_count, kMaxProfileSamples);
        profile_.range_m = terrain->range_m;
        profile_.seq = terrain->terrain_seq;
        for (std::size_t i = 0; i < profile_.samples; ++i) {
            profile_.elevation_m[i] = terrain->elevation[i].elevation;
        }
        have_profile_ = true;
        profile_unassessed_ = true;
        profile_frame_ = frame;
    }
    run_self_test(frame);
    // Nothing has to arrive before the radar is ready: it reports no guidance until it has data.
    if (status_.state == TerminalState::INITIALIZING) {
        status_.state = TerminalState::NORMAL;
    }
    status_.healthy = !bit_.failed;
    status_.mode = status_.state == TerminalState::NORMAL ? mode_ : TfrMode::NONE;

    if (!operating()) {
        if (!products_cleared_) {
            clear_products();
        }
        invalidate_guidance();
        master_ = ClearanceLevel::CLEAR;  // a new engagement starts with a clean latch
        mode_transition_ = false;
        status_.mode_transition = false;
        return;
    }

    const bool sensors_fresh = have_sensors_ && frame - sensor_frame_ < kSensorStaleFrames;
    const bool profile_fresh = have_profile_ && frame - profile_frame_ < kTerrainStaleFrames;
    if (!sensors_fresh || !profile_fresh || !usable(sensors_.altitude_ssm) || !std::isfinite(sensors_.altitude)) {
        invalidate_guidance();
        status_.mode_transition = mode_transition_;
        return;
    }
    const double speed_mps = usable(sensors_.airspeed_ssm) && std::isfinite(sensors_.airspeed)
                                 ? std::fmax(kMinSpeedMps, sensors_.airspeed * kMpsPerKt)
                                 : kDefaultSpeedMps;
    assess(sensors_.altitude, speed_mps);
    mode_transition_ = false;
    status_.mode_transition = false;
    if (profile_unassessed_ && (!block_published_ || frame - last_block_frame_ >= kElevationBlockFrames)) {
        fill_products(sensors_.altitude);
        products_dirty_ = true;
        products_cleared_ = false;
        block_published_ = true;
        last_block_frame_ = frame;
        profile_unassessed_ = false;
    }
}

void TerrainFollowing::assess(double altitude_ft, double speed_mps) noexcept {
    const double altitude_m = altitude_ft * kMPerFt;
    assess_profile(profile_, altitude_m, master_, assessment_);
    master_ = assessment_.master;
    compute_path(profile_, altitude_m, speed_mps, set_clearance_ft_ * kMPerFt, path_);
    ++profiles_assessed_;
    fill_guidance(altitude_m);
}

void TerrainFollowing::fill_guidance(double altitude_m) noexcept {
    icd::RdrTfrGuidance g{};
    g.clearance_ssm = Ssm::NORMAL;
    g.level = assessment_.master;
    g.mode = mode_;
    g.pull_up = assessment_.master == ClearanceLevel::PULL_UP;
    std::size_t warnings = 0;
    for (std::size_t i = 0; i < profile_.samples; ++i) {
        const TerrainWarning w = terrain_warning(profile_.elevation_m[i]);
        if (w == TerrainWarning::HIGH_TERRAIN) {
            g.high_terrain = true;
        } else if (w == TerrainWarning::STEEP_TERRAIN) {
            g.steep_terrain = true;
        }
        if (w != TerrainWarning::NONE) {
            ++warnings;
        }
    }
    g.commands_valid = path_.count != 0;
    g.profile_seq = profile_.seq;
    g.warning_count = saturate8(warnings);
    g.terrain_clearance = (altitude_m - profile_.elevation_m[0]) * kFtPerM;
    g.min_clearance = assessment_.min_clearance_m * kFtPerM;
    g.min_clearance_distance = profile_.distance_m(assessment_.min_index);
    if (path_.count != 0) {
        g.commanded_altitude = std::fmax(0.0, path_.command[0].altitude_m * kFtPerM);
        g.command_distance = path_.command[0].distance_m;
    }
    g.set_clearance = set_clearance_ft_;
    guidance_ = g;
}

void TerrainFollowing::invalidate_guidance() noexcept {
    guidance_ = icd::RdrTfrGuidance{};
    guidance_.clearance_ssm = Ssm::NO_COMPUTED_DATA;
    guidance_.mode = status_.state == TerminalState::NORMAL ? mode_ : TfrMode::NONE;
    guidance_.set_clearance = set_clearance_ft_;
}

void TerrainFollowing::fill_products(double altitude_ft) noexcept {
    icd::RdrTfrElevation p{};
    p.profile_seq = profile_.seq;
    p.mode = mode_;
    p.level = assessment_.master;
    p.pull_up = assessment_.master == ClearanceLevel::PULL_UP;
    p.range_m = profile_.range_m;
    p.commands_truncated = path_.count > kMaxReportedCommands;
    p.altitude = altitude_ft;
    p.set_clearance = set_clearance_ft_;
    p.min_clearance = assessment_.min_clearance_m * kFtPerM;
    p.profile_count = static_cast<std::uint16_t>(profile_.samples);
    for (std::size_t i = 0; i < profile_.samples; ++i) {
        p.profile[i].elevation = profile_.elevation_m[i];
        p.profile[i].level = assessment_.level[i];
        p.profile[i].warning = terrain_warning(profile_.elevation_m[i]);
    }
    const std::size_t reported = std::min(path_.count, kMaxReportedCommands);
    p.commands_count = static_cast<std::uint16_t>(reported);
    for (std::size_t i = 0; i < reported; ++i) {
        p.commands[i].distance = path_.command[i].distance_m;
        p.commands[i].altitude = std::fmax(0.0, path_.command[i].altitude_m);
        p.commands[i].reason = path_.command[i].reason;
    }
    products_ = p;
}

void TerrainFollowing::clear_products() noexcept {
    products_ = icd::RdrTfrElevation{};
    products_.profile_seq = profile_.seq;
    products_.mode = TfrMode::NONE;
    products_.set_clearance = set_clearance_ft_;
    products_dirty_ = true;
    products_cleared_ = true;
    block_published_ = false;  // one segment: the next profile may follow at once
}

bool TerrainFollowing::take_products_update() noexcept {
    const bool dirty = products_dirty_;
    products_dirty_ = false;
    return dirty;
}

}  // namespace fmofp::radar
