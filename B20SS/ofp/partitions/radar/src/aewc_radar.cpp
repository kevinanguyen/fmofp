#include <fmofp/radar/aewc_radar.hpp>

#include <algorithm>
#include <cmath>

#include <fmofp/radar/target_processing.hpp>  // assess_threat
#include <fmofp/radar/weather_model.hpp>      // texture_unit: the repeatable uniform draw

namespace fmofp::radar {

namespace {

using icd::AewcMode;
using icd::EcmMitigation;
using icd::JamType;
using icd::TargetClass;
using icd::TerminalState;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kMinRangeM = 1.0;
constexpr std::uint32_t kStealthDraw = 3;  // texture_unit's last argument: which draw
constexpr std::uint32_t kNoiseDrawA = 4;
constexpr std::uint32_t kNoiseDrawB = 5;

bool commandable(TerminalState s) noexcept {
    return s == TerminalState::STANDBY || s == TerminalState::NORMAL || s == TerminalState::TEST ||
           s == TerminalState::MAINTENANCE || s == TerminalState::CALIBRATION;
}

double distance_m(const std::array<double, 3>& a, const std::array<double, 3>& b) noexcept {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double magnitude(const std::array<double, 3>& v) noexcept { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// random.gauss(0, sigma) from two repeatable uniform draws (Box-Muller).
double gaussian(std::uint32_t seed, std::uint32_t pass, std::uint32_t index, double sigma) noexcept {
    const double u1 = 1.0 - texture_unit(seed, pass, index, kNoiseDrawA);  // (0, 1]
    const double u2 = texture_unit(seed, pass, index, kNoiseDrawB);
    return sigma * std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

}  // namespace

double aewc_snr_db(double rcs_m2, double range_m) noexcept {
    const double r = std::fmax(kMinRangeM, range_m);
    const double four_pi = 4.0 * kPi;
    const double received_w = (kAewcTxPowerW * kAewcAntennaGain * kAewcAntennaGain * kAewcWavelengthM * kAewcWavelengthM *
                               rcs_m2) /
                              (four_pi * four_pi * four_pi * r * r * r * r);
    if (!(received_w > 0.0)) {
        return 0.0;
    }
    const double received_db = 10.0 * std::log10(received_w);
    return std::fmax(0.0, received_db - kAewcNoiseFloorDbm);
}

std::size_t aewc_sector_of(const std::array<double, 3>& p) noexcept {
    const double horizontal = std::sqrt(p[0] * p[0] + p[1] * p[1]);
    const double elevation = std::atan2(p[2], horizontal) * kRadToDeg;
    if (!(elevation >= kAewcSectorElevationMinDeg) || !(elevation < kAewcSectorElevationMaxDeg)) {
        return 0;
    }
    double azimuth = std::fmod(std::atan2(p[1], p[0]) * kRadToDeg + 360.0, 360.0);
    if (azimuth < 0.0) {
        azimuth += 360.0;
    }
    const std::size_t index = static_cast<std::size_t>(azimuth / kAewcSectorWidthDeg);
    return index >= kAewcSectorCount ? kAewcSectorCount : index + 1;
}

TargetClass aewc_classify(double speed_mps, double altitude_m, bool stealth) noexcept {
    if (stealth) {
        return TargetClass::STEALTH;
    }
    if (speed_mps > kAewcFighterSpeedMps) {
        return TargetClass::FIGHTER;
    }
    if (std::fabs(altitude_m) > kAewcHighAltitudeM) {
        return TargetClass::HIGH_ALT;
    }
    return TargetClass::UNKNOWN;
}

EcmAssessment ecm_assess(double exceedance_db) noexcept {
    EcmAssessment a{};
    if (!(exceedance_db >= kEcmThresholdDb)) {
        return a;
    }
    a.jammed = true;
    a.severity = std::fmin(1.0, exceedance_db / kEcmSeverityFullScaleDb);
    if (exceedance_db > kEcmContinuousDb) {
        a.jam_type = JamType::CONTINUOUS;
    } else if (exceedance_db > kEcmSpotDb) {
        a.jam_type = JamType::SPOT;
    } else if (exceedance_db > kEcmSweepDb) {
        a.jam_type = JamType::SWEEP;
    } else {
        a.jam_type = JamType::DECEPTIVE;
    }
    if (a.severity < kEcmBeamNotchSeverity) {
        a.mitigation = EcmMitigation::FREQ_AGILITY;
    } else if (a.severity < kEcmPowerSeverity) {
        a.mitigation = EcmMitigation::BEAM_NOTCH;
    } else if (a.severity < kEcmSkipSeverity) {
        a.mitigation = EcmMitigation::POWER;
    } else {
        a.mitigation = EcmMitigation::SKIP;
    }
    return a;
}

SectorPriorities aewc_priorities(const std::array<std::uint8_t, kAewcSectorCount>& tracks,
                                 const std::array<std::uint8_t, kAewcSectorCount>& hostile,
                                 const std::array<double, kAewcSectorCount>& age_s, std::uint8_t jammed_mask) noexcept {
    SectorPriorities p{};
    double total = 0.0;
    for (std::size_t i = 0; i < kAewcSectorCount; ++i) {
        double score = kAewcDensityWeight * static_cast<double>(tracks[i]) + kAewcThreatWeight * static_cast<double>(hostile[i]) +
                       kAewcDwellAgeWeight * age_s[i];
        if (((static_cast<unsigned>(jammed_mask) >> i) & 1U) != 0U) {
            score += kAewcJammedBonus;
        }
        p.score[i] = score;
        total += score;
    }
    if (!(total > 0.0)) {
        total = 1.0;  // `sum(...) or 1.0`
    }
    for (std::size_t i = 0; i < kAewcSectorCount; ++i) {
        p.dwell[i] = std::fmax(kAewcMinDwell, p.score[i] / total);
        p.order[i] = static_cast<std::uint8_t>(i);
    }
    // sorted(key=-score) is stable: equal scores keep sector order (an insertion sort does too).
    for (std::size_t i = 1; i < kAewcSectorCount; ++i) {
        const std::uint8_t moving = p.order[i];
        std::size_t j = i;
        while (j > 0 && p.score[p.order[j - 1]] < p.score[moving]) {
            p.order[j] = p.order[j - 1];
            --j;
        }
        p.order[j] = moving;
    }
    return p;
}

Status AewcRadar::init() noexcept {
    status_ = icd::RdrAewcStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    block_ = icd::RdrAewcTracks{};
    block_dirty_ = true;  // the empty block is published first
    block_ready_ = false;
    block_published_ = false;
    last_block_frame_ = 0;
    mode_ = AewcMode::NONE;
    mode_seq_ = bus::CommandSequence{};
    mode_transition_ = false;
    scan_start_deg_ = 0.0;
    scan_end_deg_ = 90.0;
    std::fill(tracks_.begin(), tracks_.end(), AewcTrack{});
    std::fill(sectors_.begin(), sectors_.end(), AewcSector{});
    priority_sector_ = 0;
    next_id_ = 1;
    updates_ = 0;
    update_seq_ = 0;
    last_update_frame_ = 0;
    have_update_ = false;
    init_frame_ = 0;
    last_progress_frame_ = 0;
    have_progress_ = false;
    self_test_running_ = false;
    self_test_started_ = 0;
    bit_ = bus::BitWord{};
    return Status::kOk;
}

void AewcRadar::reset() noexcept { (void)init(); }

void AewcRadar::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void AewcRadar::run_self_test(std::uint32_t frame) noexcept {
    if (!self_test_running_) {
        return;
    }
    if (self_test_started_ == 0) {
        self_test_started_ = frame == 0 ? 1 : frame;
    } else if (frame - self_test_started_ >= kAewcSelfTestFrames) {
        self_test_running_ = false;
        self_test_started_ = 0;
        bit_.in_progress = false;
    }
}

bool AewcRadar::operating() const noexcept { return status_.state == TerminalState::NORMAL && mode_ != AewcMode::NONE; }

std::size_t AewcRadar::held_tracks() const noexcept {
    return static_cast<std::size_t>(std::count_if(tracks_.begin(), tracks_.end(), [](const AewcTrack& t) { return t.held; }));
}

std::size_t AewcRadar::stealth_tracks() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(tracks_.begin(), tracks_.end(), [](const AewcTrack& t) { return t.held && t.stealth; }));
}

const AewcTrack* AewcRadar::find(std::uint16_t id) const noexcept {
    if (id == 0) {
        return nullptr;
    }
    const auto it =
        std::find_if(tracks_.begin(), tracks_.end(), [id](const AewcTrack& t) { return t.held && t.id == id; });
    return it == tracks_.end() ? nullptr : &*it;
}

std::uint8_t AewcRadar::jammed_mask() const noexcept {
    std::uint8_t mask = 0;
    for (std::size_t i = 0; i < kAewcSectorCount; ++i) {
        if (sectors_[i].jammed) {
            mask = static_cast<std::uint8_t>(mask | (1U << i));
        }
    }
    return mask;
}

// set_mode(STANDBY): the targets, the sectors' tracks and their scan progress go.
void AewcRadar::clear_tracks() noexcept {
    std::fill(tracks_.begin(), tracks_.end(), AewcTrack{});
    for (AewcSector& s : sectors_) {
        s.active_tracks = 0;
        s.all_tracks = 0;
        s.scan_progress = 0.0;
    }
    block_dirty_ = true;
}

void AewcRadar::command_mode(const icd::RdrAewcModeCmd& cmd) noexcept {
    if (!mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.last_cmd_seq = cmd.cmd_seq;
    if (!commandable(cmd.requested_state) || !icd::is_valid(cmd.requested_mode) || !std::isfinite(cmd.azimuth_start) ||
        !std::isfinite(cmd.azimuth_end) || !std::isfinite(cmd.elevation)) {
        status_.cmd_rejected = true;
        return;
    }
    status_.cmd_rejected = false;
    status_.state = cmd.requested_state;
    const AewcMode requested = cmd.requested_state == TerminalState::NORMAL ? cmd.requested_mode : AewcMode::NONE;
    mode_transition_ = requested != mode_ && requested != AewcMode::NONE;
    mode_ = requested;
    scan_start_deg_ = cmd.azimuth_start;
    scan_end_deg_ = cmd.azimuth_end;
    if (!operating()) {
        clear_tracks();
    }
    block_dirty_ = true;
}

void AewcRadar::step(const icd::PlantTraffic* traffic, std::uint32_t frame) noexcept {
    run_self_test(frame);
    if (status_.state == TerminalState::INITIALIZING) {
        status_.state = TerminalState::NORMAL;
        init_frame_ = frame;
    }
    if (traffic != nullptr && operating()) {
        update_tracks(*traffic, frame);
    }
    if (operating()) {
        drop_stale(frame);
        advance_scan(frame);
    } else if (held_tracks() != 0) {
        clear_tracks();
    }
    status_.healthy = !bit_.failed;
    status_.mode = status_.state == TerminalState::NORMAL ? mode_ : AewcMode::NONE;
    status_.mode_transition = mode_transition_;
    fill_status();
    if (block_dirty_ && (!block_published_ || frame - last_block_frame_ >= kAewcTracksBlockFrames)) {
        fill_block();
        last_block_frame_ = frame;
        block_published_ = true;
        block_dirty_ = false;
        block_ready_ = true;
    }
}

bool AewcRadar::take_tracks_update() noexcept {
    const bool ready = block_ready_;
    block_ready_ = false;
    return ready;
}

std::size_t AewcRadar::associate(const std::array<double, 3>& position, double dt,
                                 const std::array<bool, kAewcMaxTracks>& matched) const noexcept {
    std::size_t best = kAewcMaxTracks;
    double best_distance = kAewcTrackGateM;
    for (std::size_t s = 0; s < kAewcMaxTracks; ++s) {
        const AewcTrack& track = tracks_[s];
        if (!track.held || matched[s]) {
            continue;
        }
        const std::array<double, 3> predicted{track.position[0] + track.velocity[0] * dt,
                                              track.position[1] + track.velocity[1] * dt,
                                              track.position[2] + track.velocity[2] * dt};
        const double d = distance_m(position, predicted);
        if (d < best_distance) {
            best_distance = d;
            best = s;
        }
    }
    return best;
}

std::size_t AewcRadar::new_slot() noexcept {
    const auto it = std::find_if(tracks_.begin(), tracks_.end(), [](const AewcTrack& t) { return !t.held; });
    if (it == tracks_.end()) {
        return kAewcMaxTracks;
    }
    *it = AewcTrack{};
    it->id = next_id_;
    next_id_ = static_cast<std::uint16_t>(next_id_ == 0xFFFFU ? 1U : next_id_ + 1U);
    it->held = true;
    return static_cast<std::size_t>(it - tracks_.begin());
}

// _update_targets for one RDR_TRAFFIC block: every detection within range continues
// the track it is nearest to or starts one, then _run_aewc_processing.
void AewcRadar::update_tracks(const icd::PlantTraffic& traffic, std::uint32_t frame) noexcept {
    ++updates_;
    update_seq_ = static_cast<std::uint8_t>(update_seq_ + 1U);
    const double dt = have_update_ ? static_cast<double>(frame - last_update_frame_) * kAewcFrameSeconds : 0.0;
    std::array<bool, kAewcMaxTracks> matched{};
    const std::size_t n = std::min<std::size_t>(traffic.target_count, kAewcMaxDetections);
    for (std::size_t i = 0; i < n; ++i) {
        const icd::PlantTrafficTarget& d = traffic.target[i];
        const double range_m = d.range * 1000.0;
        if (!(range_m > 0.0) || range_m > kAewcMaxRangeM) {
            continue;
        }
        const double az = d.azimuth * kDegToRad;
        const double el = d.elevation * kDegToRad;
        const std::array<double, 3> position{range_m * std::cos(el) * std::sin(az), range_m * std::cos(el) * std::cos(az),
                                             range_m * std::sin(el)};
        std::size_t s = associate(position, dt, matched);
        if (s == kAewcMaxTracks) {
            s = new_slot();
            if (s == kAewcMaxTracks) {
                continue;  // the table is full
            }
        }
        AewcTrack& track = tracks_[s];
        track.position = position;
        track.velocity = {d.velocity_east, d.velocity_north, d.velocity_up};
        track.rcs = d.rcs;
        track.last_seen_frame = frame;
        matched[s] = true;
        process_track(track, range_m);
    }
    process_sectors(frame);
    have_update_ = true;
    last_update_frame_ = frame;
    mode_transition_ = false;
    block_dirty_ = true;
}

// The Python's per-target work after the move: the SNR, the stealth detection
// draw (a failed one leaves the track where the detection put it, as the
// Python's `continue` came after the position update, and keeps it out of the
// sector counts), the classification, the sector.
void AewcRadar::process_track(AewcTrack& track, double range_m) noexcept {
    track.snr_db = aewc_snr_db(track.rcs, range_m);
    track.confidence = std::fmin(1.0, track.snr_db / kAewcSnrReferenceDb);
    track.stealth = track.rcs < kAewcStealthRcsM2;
    track.detected = true;
    if (track.stealth) {
        const double probability = kAewcStealthProbability * (track.snr_db / kAewcSnrReferenceDb);
        track.detected = !(texture_unit(seed_, track.id, updates_, kStealthDraw) > probability);
    }
    const double speed = magnitude(track.velocity);
    track.classification = aewc_classify(speed, track.position[2], track.stealth);
    track.threat = assess_threat(track.classification, track.identity, track.rcs, speed);
    const std::size_t in = aewc_sector_of(track.position);
    track.sector = static_cast<std::uint8_t>(track.detected ? in : 0U);
}

// _run_aewc_processing: the sector counts (every track; the detected ones for
// the sectors' own lists), the priority manager with last pass's jammed set,
// then the electronic protection's assessment of each sector's noise.
void AewcRadar::process_sectors(std::uint32_t frame) noexcept {
    std::array<std::uint8_t, kAewcSectorCount> counts{};
    std::array<std::uint8_t, kAewcSectorCount> detected{};
    const std::array<std::uint8_t, kAewcSectorCount> hostile{};  // nothing marked a track hostile for the manager
    for (const AewcTrack& track : tracks_) {
        if (!track.held) {
            continue;
        }
        const std::size_t in = aewc_sector_of(track.position);
        if (in != 0) {
            counts[in - 1] = static_cast<std::uint8_t>(counts[in - 1] + 1U);
            if (track.detected) {
                detected[in - 1] = static_cast<std::uint8_t>(detected[in - 1] + 1U);
            }
        }
    }
    std::array<double, kAewcSectorCount> ages{};
    const double age = static_cast<double>(frame - init_frame_) * kAewcFrameSeconds;
    ages.fill(age);
    const SectorPriorities p = aewc_priorities(counts, hostile, ages, jammed_mask());
    for (std::size_t i = 0; i < kAewcSectorCount; ++i) {
        sectors_[i].all_tracks = counts[i];
        sectors_[i].active_tracks = detected[i];
        sectors_[i].priority_score = p.score[i];
        sectors_[i].dwell_fraction = p.dwell[i];
    }
    priority_sector_ = static_cast<std::uint8_t>(p.order[0] + 1U);
    for (std::size_t i = 0; i < kAewcSectorCount; ++i) {
        const double exceedance = gaussian(seed_, updates_, static_cast<std::uint32_t>(i), kEcmNoiseSigmaDb);
        sectors_[i].ecm = ecm_assess(exceedance);
        sectors_[i].jammed = sectors_[i].ecm.jammed;
    }
}

// The sectors' scan progress: a tenth every second, round and round.
void AewcRadar::advance_scan(std::uint32_t frame) noexcept {
    if (!have_progress_) {
        have_progress_ = true;
        last_progress_frame_ = frame;
        return;
    }
    if (frame - last_progress_frame_ < kAewcScanProgressFrames) {
        return;
    }
    last_progress_frame_ = frame;
    for (AewcSector& s : sectors_) {
        s.scan_progress = std::fmod(s.scan_progress + kAewcScanProgressStep, 1.0);
    }
    block_dirty_ = true;
}

void AewcRadar::drop_stale(std::uint32_t frame) noexcept {
    for (AewcTrack& track : tracks_) {
        if (track.held && frame - track.last_seen_frame > kAewcTrackDropFrames) {
            track = AewcTrack{};
            block_dirty_ = true;
        }
    }
}

void AewcRadar::fill_status() noexcept {
    status_.track_count = static_cast<std::uint8_t>(std::min<std::size_t>(held_tracks(), kAewcMaxReportedCount));
    status_.stealth_tracks = static_cast<std::uint8_t>(std::min<std::size_t>(stealth_tracks(), kAewcMaxReportedStealth));
    status_.jammed_sectors = jammed_mask();
    status_.priority_sector = priority_sector_;
}

void AewcRadar::fill_block() noexcept {
    block_ = icd::RdrAewcTracks{};
    block_.update_seq = update_seq_;
    block_.mode = status_.mode;
    block_.scan_azimuth_start = scan_start_deg_;
    block_.scan_azimuth_end = scan_end_deg_;
    std::array<std::size_t, kAewcMaxTracks> order{};
    std::size_t n = 0;
    for (std::size_t s = 0; s < kAewcMaxTracks; ++s) {
        if (tracks_[s].held) {
            order[n++] = s;
        }
    }
    std::sort(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(n),
              [this](std::size_t a, std::size_t b) { return tracks_[a].id < tracks_[b].id; });
    for (std::size_t i = 0; i < n; ++i) {
        const AewcTrack& track = tracks_[order[i]];
        icd::RdrAewcTracksTrack& rec = block_.track[i];
        rec.track_id = track.id;
        rec.x = track.position[0];
        rec.y = track.position[1];
        rec.z = track.position[2];
        rec.velocity_east = track.velocity[0];
        rec.velocity_north = track.velocity[1];
        rec.velocity_up = track.velocity[2];
        rec.classification = track.classification;
        rec.identity = track.identity;
        rec.threat = track.threat;
        rec.stealth = track.stealth;
        rec.detected = track.detected;
        rec.sector = track.sector;
        rec.confidence = track.confidence;
        rec.snr = track.snr_db;
    }
    block_.track_count = static_cast<std::uint16_t>(n);
    for (std::size_t i = 0; i < kAewcSectorCount; ++i) {
        const AewcSector& s = sectors_[i];
        icd::RdrAewcTracksSector& rec = block_.sector[i];
        rec.azimuth_start = static_cast<double>(i) * kAewcSectorWidthDeg;
        rec.active_tracks = s.active_tracks;
        rec.jammed = s.jammed;
        rec.scan_progress = s.scan_progress;
        rec.dwell_fraction = s.dwell_fraction;
        rec.priority_score = s.priority_score;
        rec.jam_type = s.ecm.jam_type;
        rec.mitigation = s.ecm.mitigation;
        rec.severity = s.ecm.severity;
    }
    block_.sector_count = static_cast<std::uint16_t>(kAewcSectorCount);
}

}  // namespace fmofp::radar
