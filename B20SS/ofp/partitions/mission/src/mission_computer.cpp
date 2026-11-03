#include <fmofp/mission/mission_computer.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::mission {

using icd::TerminalState;

namespace {

constexpr double kPi = 3.14159265358979323846;

double radians(double deg) noexcept { return deg * kPi / 180.0; }
double degrees(double rad) noexcept { return rad * 180.0 / kPi; }

bool commandable(TerminalState s) noexcept {
    return s == TerminalState::STANDBY || s == TerminalState::NORMAL || s == TerminalState::TEST ||
           s == TerminalState::MAINTENANCE || s == TerminalState::CALIBRATION;
}

bool usable(icd::Ssm ssm) noexcept { return ssm == icd::Ssm::NORMAL; }

bool valid_position(double lat, double lon) noexcept {
    return std::isfinite(lat) && std::isfinite(lon) && lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon < 180.0;
}

bool active_target(const Target& t) noexcept {
    return t.used && t.status != icd::TargetStatus::DESTROYED && t.status != icd::TargetStatus::LOST;
}

// get_prioritised's key: (priority, -track_quality), lowest first.
bool before(const Target& a, const Target& b) noexcept {
    if (a.priority != b.priority) {
        return a.priority < b.priority;
    }
    return a.track_quality > b.track_quality;
}

}  // namespace

// ---- names -----------------------------------------------------------------------------------

NameWords pack_name(const Name& name) noexcept {
    NameWords words{};
    for (std::size_t i = 0; i < kNameWords; ++i) {
        const auto hi = static_cast<std::uint16_t>(static_cast<unsigned char>(name[2 * i]));
        const auto lo = static_cast<std::uint16_t>(static_cast<unsigned char>(name[2 * i + 1]));
        words[i] = static_cast<std::uint16_t>((hi << 8U) | lo);
    }
    return words;
}

Name unpack_name(const NameWords& words) noexcept {
    Name name{};
    for (std::size_t i = 0; i < kNameWords; ++i) {
        name[2 * i] = static_cast<char>((words[i] >> 8U) & 0xFFU);
        name[2 * i + 1] = static_cast<char>(words[i] & 0xFFU);
    }
    return name;
}

Name make_name(const char* text) noexcept {
    Name name{};
    if (text == nullptr) {
        return name;
    }
    for (std::size_t i = 0; i < kNameChars && text[i] != '\0'; ++i) {
        name[i] = text[i];
    }
    return name;
}

bool same_name(const Name& a, const Name& b) noexcept { return a == b; }

// ---- geometry --------------------------------------------------------------------------------

double haversine_nm(double lat1, double lon1, double lat2, double lon2) noexcept {
    const double phi1 = radians(lat1);
    const double phi2 = radians(lat2);
    const double dphi = radians(lat2 - lat1);
    const double dlam = radians(lon2 - lon1);
    const double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) +
                     std::cos(phi1) * std::cos(phi2) * std::sin(dlam / 2.0) * std::sin(dlam / 2.0);
    return kEarthRadiusNm * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

double bearing_deg(double lat, double lon, double to_lat, double to_lon) noexcept {
    const double dy = to_lat - lat;
    const double dx = (to_lon - lon) * std::cos(radians(lat));
    return std::fmod(degrees(std::atan2(dx, dy)) + 360.0, 360.0);
}

// ---- RouteManager ----------------------------------------------------------------------------

void RouteManager::init() noexcept {
    waypoints_ = {};
    count_ = 0;
    active_ = 0;
    next_id_ = 1;
    lat_ = 0.0;
    lon_ = 0.0;
    alt_ft_ = 0.0;
    groundspeed_kt_ = kDefaultGroundSpeedKt;
}

std::uint8_t RouteManager::add(const Name& name, double lat, double lon, double alt_ft, icd::WaypointType type,
                               bool fly_over, std::size_t index) noexcept {
    if (count_ >= kMaxWaypoints || next_id_ == 0) {
        return 0;
    }
    Waypoint wp{};
    wp.id = next_id_;
    wp.type = type;
    wp.fly_over = fly_over;
    wp.name = name;
    wp.lat = lat;
    wp.lon = lon;
    wp.alt_ft = alt_ft;
    ++next_id_;
    const std::size_t at = index == kAppend || index > count_ ? count_ : index;
    for (std::size_t i = count_; i > at; --i) {
        waypoints_[i] = waypoints_[i - 1];
    }
    waypoints_[at] = wp;
    ++count_;
    compute_etes();
    return wp.id;
}

bool RouteManager::remove(std::uint8_t id) noexcept {
    const auto end = waypoints_.begin() + static_cast<std::ptrdiff_t>(count_);
    const auto it = std::find_if(waypoints_.begin(), end, [id](const Waypoint& wp) { return wp.id == id; });
    if (it == end) {
        return false;
    }
    const auto at = static_cast<std::size_t>(it - waypoints_.begin());
    for (std::size_t i = at; i + 1 < count_; ++i) {
        waypoints_[i] = waypoints_[i + 1];
    }
    --count_;
    waypoints_[count_] = Waypoint{};
    // max(0, min(active, len - 1))
    if (count_ == 0) {
        active_ = 0;
    } else if (active_ > count_ - 1) {
        active_ = count_ - 1;
    }
    compute_etes();
    return true;
}

void RouteManager::clear() noexcept {
    waypoints_ = {};
    count_ = 0;
    active_ = 0;
}

void RouteManager::update_position(double lat, double lon, double alt_ft, double groundspeed_kt) noexcept {
    lat_ = lat;
    lon_ = lon;
    alt_ft_ = alt_ft;
    groundspeed_kt_ = std::max(kMinGroundSpeedKt, groundspeed_kt);
    if (active_ < count_) {
        const Waypoint& wp = waypoints_[active_];
        if (haversine_nm(lat, lon, wp.lat, wp.lon) < kReachedNm) {
            active_ = std::min(active_ + 1, count_);
        }
    }
    compute_etes();
}

std::uint8_t RouteManager::insert_target(const Name& name, double lat, double lon, double alt_ft) noexcept {
    std::size_t best = count_;
    double best_distance = 0.0;
    bool found = false;
    for (std::size_t i = active_; i < count_; ++i) {
        const double d = haversine_nm(waypoints_[i].lat, waypoints_[i].lon, lat, lon);
        if (!found || d < best_distance) {
            found = true;
            best_distance = d;
            best = i;
        }
    }
    return add(name, lat, lon, alt_ft, icd::WaypointType::TGT, true, best);
}

std::size_t RouteManager::deviate_from_threat(double threat_lat, double threat_lon, double radius_nm) noexcept {
    std::size_t adjusted = 0;
    for (std::size_t i = 0; i < count_; ++i) {
        Waypoint& wp = waypoints_[i];
        const double d = haversine_nm(wp.lat, wp.lon, threat_lat, threat_lon);
        if (d > 0.0 && d < radius_nm) {
            const double factor = (radius_nm - d) / d * kDeviationGain;
            wp.lat += (wp.lat - threat_lat) * factor;
            wp.lon += (wp.lon - threat_lon) * factor;
            ++adjusted;
        }
    }
    if (adjusted != 0) {
        compute_etes();
    }
    return adjusted;
}

double RouteManager::distance_to_next_nm() const noexcept {
    if (active_ >= count_) {
        return 0.0;
    }
    return haversine_nm(lat_, lon_, waypoints_[active_].lat, waypoints_[active_].lon);
}

double RouteManager::bearing_to_next_deg() const noexcept {
    if (active_ >= count_) {
        return 0.0;
    }
    return bearing_deg(lat_, lon_, waypoints_[active_].lat, waypoints_[active_].lon);
}

void RouteManager::compute_etes() noexcept {
    if (count_ == 0) {
        return;
    }
    double cumulative = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
        Waypoint& wp = waypoints_[i];
        if (i < active_) {
            wp.has_ete = false;
            wp.ete_min = 0.0;
            continue;
        }
        double d = 0.0;
        if (i == active_) {
            d = haversine_nm(lat_, lon_, wp.lat, wp.lon);
        } else {
            const Waypoint& prev = waypoints_[i - 1];
            d = haversine_nm(prev.lat, prev.lon, wp.lat, wp.lon);
        }
        cumulative += d;
        // ete_min = round((cumulative / gs) * 60, 1) if gs > 0 else None; gs is at least 1 kt here.
        wp.has_ete = true;
        wp.ete_min = std::round(cumulative / groundspeed_kt_ * 60.0 * 10.0) / 10.0;
    }
}

// ---- Targeting -------------------------------------------------------------------------------

void Targeting::init() noexcept {
    targets_ = {};
    count_ = 0;
    next_id_ = 1;
}

Target* Targeting::lookup(std::uint8_t id) noexcept {
    const std::size_t i = find(id);
    return i < kMaxTargets ? &targets_[i] : nullptr;
}

std::size_t Targeting::find(std::uint8_t id) const noexcept {
    const auto end = targets_.begin() + static_cast<std::ptrdiff_t>(count_);
    const auto it = std::find_if(targets_.begin(), end, [id](const Target& t) { return t.used && t.id == id; });
    return it == end ? kMaxTargets : static_cast<std::size_t>(it - targets_.begin());
}

std::uint8_t Targeting::add(icd::TargetKind kind, double lat, double lon, double alt_ft, std::uint8_t priority,
                            std::uint32_t frame) noexcept {
    // A freed slot is reused before the table grows.
    const auto end = targets_.begin() + static_cast<std::ptrdiff_t>(count_);
    const auto free = std::find_if(targets_.begin(), end, [](const Target& t) { return !t.used; });
    const auto slot = static_cast<std::size_t>(free - targets_.begin());
    if (slot >= kMaxTargets || next_id_ == 0) {
        return 0;
    }
    Target t{};
    t.used = true;
    t.id = next_id_;
    t.kind = kind;
    t.status = icd::TargetStatus::DESIGNATED;
    t.priority = priority;
    t.lat = lat;
    t.lon = lon;
    t.alt_ft = alt_ft;
    t.last_update_frame = frame;
    ++next_id_;
    targets_[slot] = t;
    if (slot == count_) {
        ++count_;
    }
    return t.id;
}

bool Targeting::remove(std::uint8_t id) noexcept {
    Target* t = lookup(id);
    if (t == nullptr) {
        return false;
    }
    *t = Target{};
    while (count_ > 0 && !targets_[count_ - 1].used) {
        --count_;
    }
    return true;
}

bool Targeting::acquire(std::uint8_t id, std::uint32_t frame) noexcept {
    Target* t = lookup(id);
    if (t == nullptr || t->status != icd::TargetStatus::DESIGNATED) {
        return false;
    }
    t->status = icd::TargetStatus::ACQUIRED;
    t->track_quality = kAcquiredQuality;
    t->last_update_frame = frame;
    return true;
}

bool Targeting::engage(std::uint8_t id, icd::Weapon weapon) noexcept {
    Target* t = lookup(id);
    if (t == nullptr ||
        (t->status != icd::TargetStatus::ACQUIRED && t->status != icd::TargetStatus::TRACKING)) {
        return false;
    }
    t->status = icd::TargetStatus::ENGAGED;
    t->weapon = weapon;
    return true;
}

bool Targeting::disengage(std::uint8_t id) noexcept {
    Target* t = lookup(id);
    if (t == nullptr || t->status != icd::TargetStatus::ENGAGED) {
        return false;
    }
    t->status = icd::TargetStatus::TRACKING;
    t->weapon = icd::Weapon::NONE;
    return true;
}

bool Targeting::mark_bda_pending(std::uint8_t id) noexcept {
    Target* t = lookup(id);
    if (t == nullptr || t->status != icd::TargetStatus::ENGAGED) {
        return false;
    }
    t->status = icd::TargetStatus::BDA_PENDING;
    return true;
}

bool Targeting::assess_damage(std::uint8_t id, bool destroyed) noexcept {
    Target* t = lookup(id);
    if (t == nullptr || t->status != icd::TargetStatus::BDA_PENDING) {
        return false;
    }
    t->status = destroyed ? icd::TargetStatus::DESTROYED : icd::TargetStatus::ACQUIRED;
    return true;
}

bool Targeting::update_from_radar(std::uint8_t id, double lat, double lon, double alt_ft, double speed_kts,
                                  double heading_deg, double quality, std::uint32_t frame) noexcept {
    Target* t = lookup(id);
    if (t == nullptr) {
        return false;
    }
    t->lat = lat;
    t->lon = lon;
    t->alt_ft = alt_ft;
    t->speed_kts = speed_kts;
    t->heading_deg = heading_deg;
    t->track_quality = std::min(kRadarMaxQuality, quality);
    t->last_update_frame = frame;
    if (t->status == icd::TargetStatus::DESIGNATED) {
        t->status = icd::TargetStatus::ACQUIRED;
    } else if (t->status == icd::TargetStatus::ACQUIRED) {
        t->status = icd::TargetStatus::TRACKING;
    }
    return true;
}

bool Targeting::tick(std::uint32_t frame) noexcept {
    bool any = false;
    for (std::size_t i = 0; i < count_; ++i) {
        Target& t = targets_[i];
        if (!t.used || frame <= t.last_update_frame) {
            continue;
        }
        const double stale_s = static_cast<double>(frame - t.last_update_frame) / static_cast<double>(kTickFrames);
        const double q = std::max(0.0, t.track_quality - (stale_s / kTrackDecayS) * kTrackDecayStep);
        if (q != t.track_quality) {
            t.track_quality = q;
            any = true;
        }
        if (t.track_quality < kTrackLostBelow && t.status == icd::TargetStatus::TRACKING) {
            t.status = icd::TargetStatus::LOST;
            any = true;
        }
    }
    return any;
}

std::size_t Targeting::prioritised(std::array<std::size_t, kMaxTargets>& order) const noexcept {
    std::size_t n = 0;
    for (std::size_t i = 0; i < count_; ++i) {
        if (!active_target(targets_[i])) {
            continue;
        }
        // Insertion sort keeps the order stable, as Python's sorted() does.
        std::size_t at = n;
        while (at > 0 && before(targets_[i], targets_[order[at - 1]])) {
            order[at] = order[at - 1];
            --at;
        }
        order[at] = i;
        ++n;
    }
    return n;
}

// ---- OrderOfBattle ---------------------------------------------------------------------------

void OrderOfBattle::init() noexcept {
    units_ = {};
    count_ = 0;
    next_id_ = 1;
}

std::uint8_t OrderOfBattle::add(icd::UnitKind kind, icd::Affiliation affiliation, icd::UnitStatus status,
                                std::uint8_t threat_level, double lat, double lon, double alt_ft) noexcept {
    if (count_ >= kMaxUnits || next_id_ == 0) {
        return 0;
    }
    Unit u{};
    u.id = next_id_;
    u.kind = kind;
    u.affiliation = affiliation;
    u.status = status == icd::UnitStatus::UNKNOWN ? icd::UnitStatus::OPERATIONAL : status;
    u.threat_level = threat_level;
    u.lat = lat;
    u.lon = lon;
    u.alt_ft = alt_ft;
    ++next_id_;
    units_[count_] = u;
    ++count_;
    return u.id;
}

Unit* OrderOfBattle::lookup(std::uint8_t id) noexcept {
    const auto end = units_.begin() + static_cast<std::ptrdiff_t>(count_);
    const auto it = std::find_if(units_.begin(), end, [id](const Unit& u) { return u.id == id; });
    return it == end ? nullptr : &*it;
}

bool OrderOfBattle::remove(std::uint8_t id) noexcept {
    const Unit* u = lookup(id);
    if (u == nullptr) {
        return false;
    }
    const auto at = static_cast<std::size_t>(u - units_.data());
    for (std::size_t i = at; i + 1 < count_; ++i) {
        units_[i] = units_[i + 1];
    }
    --count_;
    units_[count_] = Unit{};
    return true;
}

bool OrderOfBattle::update_position(std::uint8_t id, double lat, double lon, double alt_ft) noexcept {
    Unit* u = lookup(id);
    if (u == nullptr) {
        return false;
    }
    u->lat = lat;
    u->lon = lon;
    u->alt_ft = alt_ft;
    return true;
}

bool OrderOfBattle::update_status(std::uint8_t id, icd::UnitStatus status) noexcept {
    Unit* u = lookup(id);
    if (u == nullptr) {
        return false;
    }
    u->status = status;
    return true;
}

std::size_t OrderOfBattle::friendly() const noexcept {
    return static_cast<std::size_t>(std::count_if(units_.begin(), units_.begin() + static_cast<std::ptrdiff_t>(count_),
                                                  [](const Unit& u) { return u.affiliation == icd::Affiliation::FRIENDLY; }));
}

std::size_t OrderOfBattle::enemy() const noexcept {
    return static_cast<std::size_t>(std::count_if(units_.begin(), units_.begin() + static_cast<std::ptrdiff_t>(count_),
                                                  [](const Unit& u) { return u.affiliation == icd::Affiliation::ENEMY; }));
}

std::size_t OrderOfBattle::neutral() const noexcept { return count_ - friendly() - enemy(); }

std::size_t OrderOfBattle::threats(std::uint8_t min_level, std::array<std::size_t, kMaxUnits>& out) const noexcept {
    std::size_t n = 0;
    for (std::size_t i = 0; i < count_; ++i) {
        if (units_[i].affiliation == icd::Affiliation::ENEMY && units_[i].threat_level >= min_level) {
            out[n] = i;
            ++n;
        }
    }
    return n;
}

// ---- MissionComputer -------------------------------------------------------------------------

Status MissionComputer::init() noexcept {
    status_ = icd::MsnStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    plan_ = icd::MsnPlan{};
    route_.init();
    targeting_.init();
    oob_.init();
    phase_ = icd::MissionPhase::PLANNING;
    cmd_status_ = icd::CmdStatus::NONE;
    position_valid_ = false;
    have_flight_ = false;
    last_flight_frame_ = 0;
    frame_ = 0;
    plan_dirty_ = true;
    plan_ready_ = false;
    last_plan_frame_ = 0;
    plan_published_once_ = false;
    plan_seq_ = 0;
    self_test_started_ = 0;
    self_test_running_ = false;
    bit_ = bus::BitWord{};
    mode_seq_ = bus::CommandSequence{};
    cmd_seq_ = bus::CommandSequence{};
    fill_status();
    fill_plan();
    plan_ready_ = true;  // the first plan (an empty route) goes out at once
    plan_dirty_ = false;
    return Status::kOk;
}

void MissionComputer::reset() noexcept { (void)init(); }

void MissionComputer::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void MissionComputer::step(const icd::BcastFmsFlightData* flight, std::uint32_t frame) noexcept {
    frame_ = frame;
    if (flight != nullptr) {
        const icd::FmsFlightData& f = flight->payload;
        have_flight_ = true;
        last_flight_frame_ = frame;
        if (usable(f.position_ssm) && valid_position(f.latitude, f.longitude)) {
            const double alt = usable(f.altitude_ssm) && std::isfinite(f.altitude) ? f.altitude : 0.0;
            const double gs = usable(f.ground_speed_ssm) && std::isfinite(f.ground_speed) && f.ground_speed > 0.0
                                  ? f.ground_speed
                                  : kDefaultGroundSpeedKt;
            const std::size_t before_index = route_.active_index();
            route_.update_position(f.latitude, f.longitude, alt, gs);
            position_valid_ = true;
            if (route_.active_index() != before_index) {
                changed();
            }
        }
    }
    const bool flight_fresh = have_flight_ && frame - last_flight_frame_ < kFlightDataStaleFrames;
    if (!flight_fresh) {
        position_valid_ = false;
    }

    if (self_test_running_) {
        if (self_test_started_ == 0) {
            self_test_started_ = frame == 0 ? 1 : frame;
        } else if (frame - self_test_started_ >= kSelfTestFrames) {
            self_test_running_ = false;
            self_test_started_ = 0;
            bit_.in_progress = false;
        }
    }

    // State: INITIALIZING until the first flight data; NORMAL while it keeps coming; DEGRADED when it stops.
    if (status_.state == TerminalState::INITIALIZING && flight_fresh) {
        status_.state = TerminalState::NORMAL;
    } else if (status_.state == TerminalState::NORMAL && !flight_fresh) {
        status_.state = TerminalState::DEGRADED;
    } else if (status_.state == TerminalState::DEGRADED && flight_fresh) {
        status_.state = TerminalState::NORMAL;
    }

    // The service's once-a-second tick: the track-quality decay.
    if (frame % kTickFrames == 0 && targeting_.tick(frame)) {
        changed();
    }

    // The plan block: on a change, no sooner than kPlanMinFrames after the last publication; and every
    // kPlanRefreshFrames while a waypoint lies ahead, as its ETE moves.
    const std::uint32_t since = frame - last_plan_frame_;
    const bool refresh = route_.has_next() && since >= kPlanRefreshFrames;
    if (!plan_ready_ && ((plan_dirty_ && (!plan_published_once_ || since >= kPlanMinFrames)) || refresh)) {
        fill_plan();
        plan_ready_ = true;
        plan_dirty_ = false;
    }

    status_.healthy = !bit_.failed;
    status_.mode_transition = false;
    fill_status();
}

bool MissionComputer::take_plan_update() noexcept {
    if (!plan_ready_) {
        return false;
    }
    plan_ready_ = false;
    plan_published_once_ = true;
    last_plan_frame_ = frame_;
    return true;
}

void MissionComputer::command_mode(const icd::MsnModeCmd& cmd) noexcept {
    if (!mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.last_cmd_seq = cmd.cmd_seq;
    if (!commandable(cmd.requested_state)) {
        status_.cmd_rejected = true;
        return;
    }
    status_.cmd_rejected = false;
    status_.state = cmd.requested_state;
    status_.mode_transition = false;
}

std::size_t MissionComputer::optimise_route() noexcept {
    std::array<std::size_t, kMaxUnits> idx{};
    const std::size_t n = oob_.threats(kOptimiseMinThreatLevel, idx);
    std::size_t adjusted = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const Unit& u = oob_.unit(idx[i]);
        adjusted += route_.deviate_from_threat(u.lat, u.lon);
    }
    return adjusted;
}

void MissionComputer::command(const icd::MsnCmd& cmd) noexcept {
    if (!cmd_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    cmd_status_ = icd::CmdStatus::REJECTED_INVALID;
    const auto id = static_cast<std::uint8_t>(cmd.id);
    const bool position_ok = valid_position(cmd.latitude, cmd.longitude) && std::isfinite(cmd.altitude);
    switch (cmd.command) {
        case icd::MissionCommand::SET_PHASE:
            if (icd::is_valid(cmd.phase)) {
                phase_ = cmd.phase;
                cmd_status_ = icd::CmdStatus::ACCEPTED;
                changed();
            }
            break;
        case icd::MissionCommand::ADVANCE_PHASE:
            if (phase_ != icd::MissionPhase::COMPLETE) {
                phase_ = static_cast<icd::MissionPhase>(static_cast<std::uint8_t>(phase_) + 1U);
            }
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            changed();
            break;
        case icd::MissionCommand::ADD_WAYPOINT:
            if (position_ok && icd::is_valid(cmd.waypoint_type)) {
                const NameWords words = {static_cast<std::uint16_t>(cmd.name_0), static_cast<std::uint16_t>(cmd.name_1),
                                         static_cast<std::uint16_t>(cmd.name_2), static_cast<std::uint16_t>(cmd.name_3)};
                const std::uint8_t added = route_.add(unpack_name(words), cmd.latitude, cmd.longitude, cmd.altitude,
                                                      cmd.waypoint_type, cmd.fly_over);
                cmd_status_ = added != 0 ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
                changed();
            }
            break;
        case icd::MissionCommand::REMOVE_WAYPOINT:
            cmd_status_ = route_.remove(id) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
            changed();
            break;
        case icd::MissionCommand::CLEAR_ROUTE:
            route_.clear();
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            changed();
            break;
        case icd::MissionCommand::DESIGNATE_TARGET:
            if (position_ok && icd::is_valid(cmd.target_kind)) {
                const std::uint8_t added = targeting_.add(cmd.target_kind, cmd.latitude, cmd.longitude, cmd.altitude,
                                                          static_cast<std::uint8_t>(cmd.priority), frame_);
                if (added != 0) {
                    // designate_target inserts a TGT waypoint named after the target.
                    char text[kNameChars + 1] = "TGT-0000";
                    text[4] = static_cast<char>('0' + (added / 1000U) % 10U);
                    text[5] = static_cast<char>('0' + (added / 100U) % 10U);
                    text[6] = static_cast<char>('0' + (added / 10U) % 10U);
                    text[7] = static_cast<char>('0' + added % 10U);
                    (void)route_.insert_target(make_name(text), cmd.latitude, cmd.longitude, cmd.altitude);
                    cmd_status_ = icd::CmdStatus::ACCEPTED;
                } else {
                    cmd_status_ = icd::CmdStatus::FAILED;
                }
                changed();
            }
            break;
        case icd::MissionCommand::ACQUIRE_TARGET:
            cmd_status_ = targeting_.acquire(id, frame_) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
            changed();
            break;
        case icd::MissionCommand::ENGAGE_TARGET:
            if (icd::is_valid(cmd.weapon) && cmd.weapon != icd::Weapon::NONE) {
                cmd_status_ = targeting_.engage(id, cmd.weapon) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
                changed();
            }
            break;
        case icd::MissionCommand::DISENGAGE_TARGET:
            cmd_status_ = targeting_.disengage(id) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
            changed();
            break;
        case icd::MissionCommand::ASSESS_DAMAGE: {
            const std::size_t i = targeting_.find(id);
            if (i < kMaxTargets && targeting_.target(i).status == icd::TargetStatus::ENGAGED) {
                (void)targeting_.mark_bda_pending(id);
            }
            cmd_status_ = targeting_.assess_damage(id, cmd.destroyed) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
            changed();
            break;
        }
        case icd::MissionCommand::REMOVE_TARGET:
            cmd_status_ = targeting_.remove(id) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
            changed();
            break;
        case icd::MissionCommand::ADD_UNIT:
            if (position_ok && icd::is_valid(cmd.unit_kind) && icd::is_valid(cmd.affiliation) &&
                icd::is_valid(cmd.unit_status) && cmd.threat_level <= 5U) {
                const std::uint8_t added =
                    oob_.add(cmd.unit_kind, cmd.affiliation, cmd.unit_status, static_cast<std::uint8_t>(cmd.threat_level),
                             cmd.latitude, cmd.longitude, cmd.altitude);
                cmd_status_ = added != 0 ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
                changed();
            }
            break;
        case icd::MissionCommand::REMOVE_UNIT:
            cmd_status_ = oob_.remove(id) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
            changed();
            break;
        case icd::MissionCommand::SET_UNIT_STATUS:
            if (icd::is_valid(cmd.unit_status)) {
                cmd_status_ = oob_.update_status(id, cmd.unit_status) ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
                changed();
            }
            break;
        case icd::MissionCommand::OPTIMISE_ROUTE:
            (void)optimise_route();
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            changed();
            break;
        case icd::MissionCommand::NONE:
            break;
    }
    status_.mission_cmd_status = cmd_status_;
    fill_status();
}

void MissionComputer::fill_status() noexcept {
    status_.phase = phase_;
    status_.mission_cmd_status = cmd_status_;
    status_.position_valid = position_valid_;
    status_.route_active = route_.has_next();
    status_.waypoint_count = static_cast<std::uint8_t>(route_.count());
    status_.active_index = static_cast<std::uint8_t>(route_.active_index());
    std::array<std::size_t, kMaxTargets> order{};
    status_.target_count = static_cast<std::uint8_t>(targeting_.prioritised(order));
    status_.friendly_units = static_cast<std::uint8_t>(oob_.friendly());
    status_.enemy_units = static_cast<std::uint8_t>(oob_.enemy());
    status_.neutral_units = static_cast<std::uint8_t>(oob_.neutral());
    status_.distance_to_next = route_.distance_to_next_nm();
    status_.bearing_to_next = route_.bearing_to_next_deg();
}

void MissionComputer::fill_plan() noexcept {
    plan_ = icd::MsnPlan{};
    plan_seq_ = static_cast<std::uint8_t>(plan_seq_ == 255U ? 1U : plan_seq_ + 1U);
    plan_.plan_seq = plan_seq_;
    plan_.phase = phase_;
    plan_.active_index = static_cast<std::uint8_t>(route_.active_index());
    plan_.distance_to_next = route_.distance_to_next_nm();
    plan_.bearing_to_next = route_.bearing_to_next_deg();
    for (std::size_t i = 0; i < route_.count(); ++i) {
        const Waypoint& wp = route_.waypoint(i);
        icd::MsnPlanWaypoint& r = plan_.waypoint[i];
        r.id = wp.id;
        r.type = wp.type;
        r.fly_over = wp.fly_over;
        r.passed = i < route_.active_index();
        const NameWords words = pack_name(wp.name);
        r.name_0 = words[0];
        r.name_1 = words[1];
        r.name_2 = words[2];
        r.name_3 = words[3];
        r.latitude = wp.lat;
        r.longitude = wp.lon;
        r.altitude = wp.alt_ft;
        r.ete = wp.has_ete ? wp.ete_min : 0.0;
    }
    plan_.waypoint_count = static_cast<std::uint16_t>(route_.count());
    std::array<std::size_t, kMaxTargets> order{};
    const std::size_t n_targets = targeting_.prioritised(order);
    for (std::size_t i = 0; i < n_targets; ++i) {
        const Target& t = targeting_.target(order[i]);
        icd::MsnPlanTarget& r = plan_.target[i];
        r.id = t.id;
        r.kind = t.kind;
        r.status = t.status;
        r.priority = t.priority;
        r.weapon = t.weapon;
        r.track_quality = t.track_quality;
        r.latitude = t.lat;
        r.longitude = t.lon;
        r.altitude = t.alt_ft;
        r.speed = t.speed_kts;
        r.heading = t.heading_deg;
    }
    plan_.target_count = static_cast<std::uint16_t>(n_targets);
    for (std::size_t i = 0; i < oob_.count(); ++i) {
        const Unit& u = oob_.unit(i);
        icd::MsnPlanUnit& r = plan_.unit[i];
        r.id = u.id;
        r.kind = u.kind;
        r.affiliation = u.affiliation;
        r.status = u.status;
        r.threat_level = u.threat_level;
        r.heading = u.heading_deg;
        r.latitude = u.lat;
        r.longitude = u.lon;
        r.altitude = u.alt_ft;
        r.speed = u.speed_kts;
    }
    plan_.unit_count = static_cast<std::uint16_t>(oob_.count());
}

}  // namespace fmofp::mission
