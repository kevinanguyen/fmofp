#include <fmofp/radar/targeting_radar.hpp>

#include <algorithm>
#include <cmath>

#include <fmofp/radar/weather_model.hpp>  // texture_unit: the repeatable uniform draw

namespace fmofp::radar {

namespace {

using icd::LockStatus;
using icd::TargetingMode;
using icd::TerminalState;
using icd::TrackFilter;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMinRangeM = 1.0;  // the SNR model's log needs a range
constexpr std::uint32_t kScintillationDraw = 1;
constexpr std::uint32_t kJammingDraw = 2;

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

// targeting_radar._update_targets: snr = 30 - 20 log10(range / 1000).
double snr_for(double range_m) noexcept { return kTgtSnrReferenceDb - 20.0 * std::log10(std::fmax(kMinRangeM, range_m) / 1000.0); }

// min(1.0, snr / 30), and never below 0 (the Python's could be; the ICD's field cannot).
double quality_of(double snr_db) noexcept { return std::fmax(0.0, std::fmin(1.0, snr_db / kTgtSnrReferenceDb)); }

}  // namespace

Status TargetingRadar::init() noexcept {
    status_ = icd::RdrTgtStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    status_.track_filter = TrackFilter::ALL;
    block_ = icd::RdrTgtTracks{};
    block_dirty_ = true;  // the empty block is published first
    block_ready_ = false;
    block_published_ = false;
    last_block_frame_ = 0;
    mode_ = TargetingMode::NONE;
    filter_ = TrackFilter::ALL;
    mode_seq_ = bus::CommandSequence{};
    mode_transition_ = false;
    std::fill(tracks_.begin(), tracks_.end(), TgtTrack{});
    next_id_ = 1;
    locked_id_ = 0;
    lock_quality_ = 0.0;
    jamming_ = false;
    updates_ = 0;
    update_seq_ = 0;
    last_update_frame_ = 0;
    have_update_ = false;
    self_test_running_ = false;
    self_test_started_ = 0;
    bit_ = bus::BitWord{};
    return Status::kOk;
}

void TargetingRadar::reset() noexcept { (void)init(); }

void TargetingRadar::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void TargetingRadar::run_self_test(std::uint32_t frame) noexcept {
    if (!self_test_running_) {
        return;
    }
    if (self_test_started_ == 0) {
        self_test_started_ = frame == 0 ? 1 : frame;
    } else if (frame - self_test_started_ >= kTgtSelfTestFrames) {
        self_test_running_ = false;
        self_test_started_ = 0;
        bit_.in_progress = false;  // nothing in this partition can fail yet
    }
}

bool TargetingRadar::operating() const noexcept {
    return status_.state == TerminalState::NORMAL && mode_ != TargetingMode::NONE;
}

std::size_t TargetingRadar::held_tracks() const noexcept {
    return static_cast<std::size_t>(
        std::count_if(tracks_.begin(), tracks_.end(), [](const TgtTrack& t) { return t.held; }));
}

const TgtTrack* TargetingRadar::find(std::uint16_t id) const noexcept {
    if (id == 0) {
        return nullptr;
    }
    const auto it =
        std::find_if(tracks_.begin(), tracks_.end(), [id](const TgtTrack& t) { return t.held && t.id == id; });
    return it == tracks_.end() ? nullptr : &*it;
}

bool TargetingRadar::passes_filter(const TgtTrack& track) const noexcept {
    switch (filter_) {
        case TrackFilter::ALL:
            return true;
        case TrackFilter::HOSTILE:
            return track.threat == icd::Threat::HOSTILE;
        case TrackFilter::FRIENDLY:
            return track.threat == icd::Threat::FRIENDLY;
        case TrackFilter::UNKNOWN:
            return track.threat == icd::Threat::UNKNOWN;
        default:
            return false;
    }
}

void TargetingRadar::release_lock() noexcept {
    locked_id_ = 0;
    lock_quality_ = 0.0;
    jamming_ = false;
}

void TargetingRadar::clear_tracks() noexcept {
    std::fill(tracks_.begin(), tracks_.end(), TgtTrack{});
    release_lock();
    block_dirty_ = true;
}

// _handle_lock_request: a lock on a track the radar holds; one it does not is
// logged and ignored by the Python. 0 releases the lock (new to the ICD).
void TargetingRadar::apply_lock_request(std::uint16_t id) noexcept {
    if (id == 0) {
        release_lock();
        return;
    }
    const TgtTrack* track = find(id);
    if (track == nullptr) {
        return;
    }
    locked_id_ = id;
    lock_quality_ = quality_of(track->snr_db);
    jamming_ = false;
}

void TargetingRadar::command_mode(const icd::RdrTgtModeCmd& cmd) noexcept {
    if (!mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.last_cmd_seq = cmd.cmd_seq;
    if (!commandable(cmd.requested_state) || !icd::is_valid(cmd.requested_mode) || !icd::is_valid(cmd.track_filter)) {
        status_.cmd_rejected = true;
        return;
    }
    status_.cmd_rejected = false;
    status_.state = cmd.requested_state;
    const TargetingMode requested =
        cmd.requested_state == TerminalState::NORMAL ? cmd.requested_mode : TargetingMode::NONE;
    mode_transition_ = requested != mode_ && requested != TargetingMode::NONE;
    mode_ = requested;
    filter_ = cmd.track_filter;
    if (!operating()) {
        clear_tracks();  // set_mode(STANDBY) cleared the targets and the lock
    } else if (mode_ == TargetingMode::TARGET_TRACK) {
        apply_lock_request(cmd.lock_track_id);  // the Python refused lock requests in any other mode
    }
    block_dirty_ = true;
}

void TargetingRadar::step(const icd::PlantTraffic* traffic, std::uint32_t frame) noexcept {
    run_self_test(frame);
    // Nothing has to arrive before the radar is ready: it holds no tracks until it detects something.
    if (status_.state == TerminalState::INITIALIZING) {
        status_.state = TerminalState::NORMAL;
    }
    if (traffic != nullptr && operating()) {
        update_tracks(*traffic, frame);
    }
    if (operating()) {
        drop_stale(frame);
    } else if (held_tracks() != 0) {
        clear_tracks();
    }
    status_.healthy = !bit_.failed;
    status_.mode = status_.state == TerminalState::NORMAL ? mode_ : TargetingMode::NONE;
    status_.mode_transition = mode_transition_;
    fill_status();
    if (block_dirty_ && (!block_published_ || frame - last_block_frame_ >= kTgtTracksBlockFrames)) {
        fill_block();
        last_block_frame_ = frame;
        block_published_ = true;
        block_dirty_ = false;
        block_ready_ = true;
    }
}

bool TargetingRadar::take_tracks_update() noexcept {
    const bool ready = block_ready_;
    block_ready_ = false;
    return ready;
}

// The held, unmatched track whose predicted position is nearest within the
// gate, or kTgtMaxTracks when there is none.
std::size_t TargetingRadar::associate(const std::array<double, 3>& position, double dt,
                                      const std::array<bool, kTgtMaxTracks>& matched) const noexcept {
    std::size_t best = kTgtMaxTracks;
    double best_distance = kTgtTrackGateM;
    for (std::size_t s = 0; s < kTgtMaxTracks; ++s) {
        const TgtTrack& track = tracks_[s];
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

std::size_t TargetingRadar::new_slot() noexcept {
    const auto it = std::find_if(tracks_.begin(), tracks_.end(), [](const TgtTrack& t) { return !t.held; });
    if (it == tracks_.end()) {
        return kTgtMaxTracks;
    }
    *it = TgtTrack{};
    it->id = next_id_;
    next_id_ = static_cast<std::uint16_t>(next_id_ == 0xFFFFU ? 1U : next_id_ + 1U);
    it->held = true;
    return static_cast<std::size_t>(it - tracks_.begin());
}

// _update_targets and _run_target_processing for one RDR_TRAFFIC block: each
// detection within range continues the track it is nearest to, or starts one.
void TargetingRadar::update_tracks(const icd::PlantTraffic& traffic, std::uint32_t frame) noexcept {
    ++updates_;
    update_seq_ = static_cast<std::uint8_t>(update_seq_ + 1U);
    const double dt = have_update_ ? static_cast<double>(frame - last_update_frame_) * kTgtFrameSeconds : 0.0;
    std::array<bool, kTgtMaxTracks> matched{};
    const std::size_t n = std::min<std::size_t>(traffic.target_count, kTgtMaxDetections);
    for (std::size_t i = 0; i < n; ++i) {
        const icd::PlantTrafficTarget& d = traffic.target[i];
        const double range_m = d.range * 1000.0;
        if (!(range_m > 0.0) || range_m > kTgtMaxRangeM) {
            continue;  // beyond max_range the Python dropped the target; NaN never reaches a track
        }
        const double az = d.azimuth * kDegToRad;
        const double el = d.elevation * kDegToRad;
        const std::array<double, 3> position{range_m * std::cos(el) * std::sin(az), range_m * std::cos(el) * std::cos(az),
                                             range_m * std::sin(el)};
        std::size_t s = associate(position, dt, matched);
        if (s == kTgtMaxTracks) {
            s = new_slot();
            if (s == kTgtMaxTracks) {
                continue;  // the table is full
            }
        }
        TgtTrack& track = tracks_[s];
        track.position = position;
        track.velocity = {d.velocity_east, d.velocity_north, d.velocity_up};
        track.rcs = d.rcs;
        track.snr_db = snr_for(range_m);
        track.last_seen_frame = frame;
        matched[s] = true;
        process_track(track);
    }
    update_lock();
    have_update_ = true;
    last_update_frame_ = frame;
    mode_transition_ = false;
    block_dirty_ = true;
}

// SignatureAnalyzer.analyse, StealthDetector.assess and the threat rule for one track after a detection.
void TargetingRadar::process_track(TgtTrack& track) noexcept {
    const double speed = magnitude(track.velocity);
    const double u = texture_unit(seed_, track.id, updates_, kScintillationDraw);  // [0, 1)
    const double scintillation = -std::log(1.0 - u);                                 // random.expovariate(1.0)
    const SignatureResult signature = analyse_signature(track.rcs, scintillation, speed, track.position[2]);
    track.classification = signature.type;
    track.detection_confidence = signature.confidence;
    track.rcs_corrected = signature.rcs_corrected;
    const StealthResult stealth = assess_stealth(track.stealth, signature.rcs_corrected, track.snr_db);
    track.stealth_probability = stealth.posterior;
    track.is_stealth = stealth.is_stealth;
    track.confidence = quality_of(track.snr_db);
    track.threat = assess_threat(track.classification, track.identity, track.rcs, speed);
}

void TargetingRadar::drop_stale(std::uint32_t frame) noexcept {
    for (TgtTrack& track : tracks_) {
        if (track.held && frame - track.last_seen_frame > kTgtTrackDropFrames) {
            if (track.id == locked_id_) {
                release_lock();
            }
            track = TgtTrack{};
            block_dirty_ = true;
        }
    }
}

// The lock quality of the locked track and the Python's 5 % jamming draw; a lost track releases the lock.
void TargetingRadar::update_lock() noexcept {
    if (locked_id_ == 0) {
        return;
    }
    const TgtTrack* track = find(locked_id_);
    if (track == nullptr) {
        release_lock();
        return;
    }
    lock_quality_ = quality_of(track->snr_db);
    jamming_ = texture_unit(seed_, locked_id_, updates_, kJammingDraw) < kTgtJammingProbability;
}

void TargetingRadar::fill_status() noexcept {
    const std::size_t held = held_tracks();
    status_.track_count = static_cast<std::uint8_t>(std::min<std::size_t>(held, kTgtMaxReportedCount));
    status_.lock_status = locked_id_ == 0 ? LockStatus::NONE
                          : lock_quality_ > kTgtLockedQuality ? LockStatus::LOCKED
                                                              : LockStatus::ACQUIRING;
    status_.jamming = jamming_;
    status_.track_filter = filter_;
    status_.locked_track_id = locked_id_;
    status_.lock_quality = lock_quality_;
}

void TargetingRadar::fill_block() noexcept {
    block_ = icd::RdrTgtTracks{};
    block_.update_seq = update_seq_;
    block_.mode = status_.mode;
    block_.lock_status = status_.lock_status;
    block_.jamming = jamming_;
    block_.locked_track_id = locked_id_;
    block_.lock_quality = lock_quality_;
    std::array<std::size_t, kTgtMaxTracks> order{};
    std::size_t n = 0;
    for (std::size_t s = 0; s < kTgtMaxTracks; ++s) {
        if (tracks_[s].held && passes_filter(tracks_[s])) {
            order[n++] = s;
        }
    }
    std::sort(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(n),
              [this](std::size_t a, std::size_t b) { return tracks_[a].id < tracks_[b].id; });
    const std::size_t reported = std::min(n, static_cast<std::size_t>(icd::RdrTgtTracks::kMaxTrack));
    for (std::size_t i = 0; i < reported; ++i) {
        const TgtTrack& track = tracks_[order[i]];
        icd::RdrTgtTracksTrack& rec = block_.track[i];
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
        rec.stealth = track.is_stealth;
        rec.locked = track.id == locked_id_;
        rec.confidence = track.confidence;
        rec.stealth_probability = track.stealth_probability;
    }
    block_.track_count = static_cast<std::uint16_t>(reported);
}

}  // namespace fmofp::radar
