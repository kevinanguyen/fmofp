#ifndef FMOFP_MISSION_MISSION_COMPUTER_HPP
#define FMOFP_MISSION_MISSION_COMPUTER_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/icd/messages.hpp>

// The mission computer's logic (RT 6), ported from Systems/missionPlanning:
// handleRoute.py's RouteManager (the route, its progress against the aircraft's
// position, the ETEs, target insertion, threat deviation), targeting.py's
// Targeting (designated targets, their status machine and the track-quality
// decay), orderOfBattle.py's OrderOfBattle (friendly, enemy and neutral units),
// and the service around them (missionService.py, missionControl.py's
// MissionControl: the phase, the position from the FMS, the route optimisation
// around threats, the snapshot the displays read). Pure: ICD messages in, ICD
// messages and the plan block out; the tests drive it frame by frame.

namespace fmofp::mission {

constexpr std::size_t kMaxWaypoints = icd::MsnPlan::kMaxWaypoint;
constexpr std::size_t kMaxTargets = icd::MsnPlan::kMaxTarget;
constexpr std::size_t kMaxUnits = icd::MsnPlan::kMaxUnit;
constexpr std::size_t kNameChars = 8;
constexpr std::size_t kNameWords = 4;
constexpr std::size_t kAppend = static_cast<std::size_t>(-1);

// handleRoute.py
constexpr double kEarthRadiusNm = 3440.065;      // _haversine_nm's R_NM
constexpr double kReachedNm = 0.5;               // RouteManager._REACHED_NM
constexpr double kDefaultGroundSpeedKt = 450.0;  // update_position's default, and the service's when the FMS had none
constexpr double kMinGroundSpeedKt = 1.0;        // max(1.0, groundspeed_kts)
constexpr double kThreatRadiusNm = 20.0;         // deviate_from_threat's default radius
constexpr double kDeviationGain = 1.1;
// missionControl.py MissionControl.optimise_route_for_threats: enemy units at or above this level.
constexpr std::uint8_t kOptimiseMinThreatLevel = 2;
// targeting.py
constexpr double kTrackDecayS = 30.0;     // Targeting._TRACK_DECAY_S
constexpr double kTrackDecayStep = 0.05;  // simulate_tick: quality -= stale / 30 * 0.05 per tick
constexpr double kTrackLostBelow = 0.1;
constexpr double kAcquiredQuality = 0.5;
constexpr double kRadarMaxQuality = 1.0;
// The service's loop ran once a second: the targeting tick, at 50 frames a second.
constexpr std::uint32_t kTickFrames = 50;
constexpr std::uint32_t kFlightDataStaleFrames = 15;  // BCAST_FMS_FLIGHT_DATA is 10 Hz: three periods
// The plan block: thirteen segments at 10 Hz take 65 frames. Republished no sooner than kPlanMinFrames after
// a change, and every kPlanRefreshFrames while a route is being flown (its ETEs move).
constexpr std::uint32_t kPlanMinFrames = 75;
constexpr std::uint32_t kPlanRefreshFrames = 150;
constexpr std::uint32_t kSelfTestFrames = 10;

using Name = std::array<char, kNameChars>;
using NameWords = std::array<std::uint16_t, kNameWords>;

// Eight ASCII characters in four words, the first of each pair in the high byte, 0 padding.
[[nodiscard]] NameWords pack_name(const Name& name) noexcept;
[[nodiscard]] Name unpack_name(const NameWords& words) noexcept;
[[nodiscard]] Name make_name(const char* text) noexcept;  // the first eight characters, 0-padded
[[nodiscard]] bool same_name(const Name& a, const Name& b) noexcept;

// _haversine_nm: the great-circle distance in nautical miles.
[[nodiscard]] double haversine_nm(double lat1, double lon1, double lat2, double lon2) noexcept;
// get_nav_data's bearing: atan2 of the longitude difference scaled by cos(latitude) over the latitude difference.
[[nodiscard]] double bearing_deg(double lat, double lon, double to_lat, double to_lon) noexcept;

struct Waypoint {
    std::uint8_t id{};
    icd::WaypointType type{icd::WaypointType::NAV};
    bool fly_over{true};
    Name name{};
    double lat{};
    double lon{};
    double alt_ft{};
    double ete_min{};     // meaningful when has_ete
    bool has_ete{false};  // None in the Python: already passed, or no route
};

// RouteManager: the ordered route, its active waypoint and the ETEs.
class RouteManager {
public:
    void init() noexcept;
    // add_waypoint: appended, or inserted before `index`. Returns the id, 0 when the route is full.
    std::uint8_t add(const Name& name, double lat, double lon, double alt_ft, icd::WaypointType type, bool fly_over,
                     std::size_t index = kAppend) noexcept;
    bool remove(std::uint8_t id) noexcept;  // remove_waypoint
    void clear() noexcept;                  // clear_route
    // update_position: the active waypoint is reached within 0.5 nm (one per call); then the ETEs.
    void update_position(double lat, double lon, double alt_ft, double groundspeed_kt) noexcept;
    // insert_target: a TGT waypoint before the nearest waypoint ahead; appended when none lies ahead.
    std::uint8_t insert_target(const Name& name, double lat, double lon, double alt_ft) noexcept;
    // deviate_from_threat: the waypoints inside the radius are pushed radially outward. Returns how many moved.
    std::size_t deviate_from_threat(double threat_lat, double threat_lon,
                                    double radius_nm = kThreatRadiusNm) noexcept;

    [[nodiscard]] std::size_t count() const noexcept { return count_; }
    [[nodiscard]] std::size_t active_index() const noexcept { return active_; }
    [[nodiscard]] bool has_next() const noexcept { return active_ < count_; }
    [[nodiscard]] const Waypoint& waypoint(std::size_t i) const noexcept { return waypoints_[i]; }
    [[nodiscard]] std::size_t remaining() const noexcept { return count_ - active_; }
    [[nodiscard]] double distance_to_next_nm() const noexcept;  // 0 without a waypoint ahead
    [[nodiscard]] double bearing_to_next_deg() const noexcept;  // 0 without a waypoint ahead

private:
    void compute_etes() noexcept;

    std::array<Waypoint, kMaxWaypoints> waypoints_{};
    std::size_t count_{0};
    std::size_t active_{0};
    std::uint8_t next_id_{1};
    double lat_{0.0};
    double lon_{0.0};
    double alt_ft_{0.0};
    double groundspeed_kt_{kDefaultGroundSpeedKt};
};

struct Target {
    bool used{false};
    std::uint8_t id{};
    icd::TargetKind kind{icd::TargetKind::UNKNOWN};
    icd::TargetStatus status{icd::TargetStatus::DESIGNATED};
    std::uint8_t priority{3};
    icd::Weapon weapon{icd::Weapon::NONE};
    double lat{};
    double lon{};
    double alt_ft{};
    double track_quality{0.0};
    double speed_kts{0.0};
    double heading_deg{0.0};
    std::uint32_t last_update_frame{0};  // Target.last_updated, in bus frames
};

// Targeting: the designated targets and their status machine.
class Targeting {
public:
    void init() noexcept;
    std::uint8_t add(icd::TargetKind kind, double lat, double lon, double alt_ft, std::uint8_t priority,
                     std::uint32_t frame) noexcept;  // add_target; 0 when the table is full
    bool remove(std::uint8_t id) noexcept;
    bool acquire(std::uint8_t id, std::uint32_t frame) noexcept;                     // DESIGNATED -> ACQUIRED, 0.5
    bool engage(std::uint8_t id, icd::Weapon weapon) noexcept;                       // ACQUIRED/TRACKING -> ENGAGED
    bool disengage(std::uint8_t id) noexcept;                                        // ENGAGED -> TRACKING
    bool mark_bda_pending(std::uint8_t id) noexcept;                                 // ENGAGED -> BDA_PENDING
    bool assess_damage(std::uint8_t id, bool destroyed) noexcept;                    // BDA_PENDING -> DESTROYED / ACQUIRED
    bool update_from_radar(std::uint8_t id, double lat, double lon, double alt_ft, double speed_kts,
                           double heading_deg, double quality, std::uint32_t frame) noexcept;
    // simulate_tick: every target's quality decays with its staleness; a tracked one below 0.1 is lost.
    // Returns true when anything changed.
    bool tick(std::uint32_t frame) noexcept;
    // get_prioritised: the indices of the targets neither destroyed nor lost, by priority then by quality
    // (highest first). Returns how many.
    std::size_t prioritised(std::array<std::size_t, kMaxTargets>& order) const noexcept;

    [[nodiscard]] std::size_t count() const noexcept { return count_; }  // slots in use, including destroyed and lost
    [[nodiscard]] const Target& target(std::size_t i) const noexcept { return targets_[i]; }
    [[nodiscard]] std::size_t find(std::uint8_t id) const noexcept;  // kMaxTargets when absent

private:
    Target* lookup(std::uint8_t id) noexcept;

    std::array<Target, kMaxTargets> targets_{};
    std::size_t count_{0};
    std::uint8_t next_id_{1};
};

struct Unit {
    std::uint8_t id{};
    icd::UnitKind kind{icd::UnitKind::UNKNOWN};
    icd::Affiliation affiliation{icd::Affiliation::UNKNOWN};
    icd::UnitStatus status{icd::UnitStatus::OPERATIONAL};
    std::uint8_t threat_level{0};
    double lat{};
    double lon{};
    double alt_ft{};
    double speed_kts{0.0};
    double heading_deg{0.0};
};

// OrderOfBattle: the units by affiliation (an unknown affiliation counts with the neutral forces).
class OrderOfBattle {
public:
    void init() noexcept;
    std::uint8_t add(icd::UnitKind kind, icd::Affiliation affiliation, icd::UnitStatus status,
                     std::uint8_t threat_level, double lat, double lon, double alt_ft) noexcept;  // 0 when full
    bool remove(std::uint8_t id) noexcept;
    bool update_position(std::uint8_t id, double lat, double lon, double alt_ft) noexcept;
    bool update_status(std::uint8_t id, icd::UnitStatus status) noexcept;

    [[nodiscard]] std::size_t count() const noexcept { return count_; }
    [[nodiscard]] const Unit& unit(std::size_t i) const noexcept { return units_[i]; }
    [[nodiscard]] std::size_t friendly() const noexcept;
    [[nodiscard]] std::size_t enemy() const noexcept;
    [[nodiscard]] std::size_t neutral() const noexcept;  // neutral and unknown
    // get_threats: enemy units at or above the level. Returns how many; their indices in `out`.
    std::size_t threats(std::uint8_t min_level, std::array<std::size_t, kMaxUnits>& out) const noexcept;

private:
    Unit* lookup(std::uint8_t id) noexcept;

    std::array<Unit, kMaxUnits> units_{};
    std::size_t count_{0};
    std::uint8_t next_id_{1};
};

class MissionComputer {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame: the FMS flight data heard by broadcast this frame, or null; the frame count.
    void step(const icd::BcastFmsFlightData* flight, std::uint32_t frame) noexcept;

    // Commands, each applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::MsnModeCmd& cmd) noexcept;
    void command(const icd::MsnCmd& cmd) noexcept;

    // Mode code effects, relayed by the process from its RemoteTerminal.
    void reset() noexcept;
    void start_self_test() noexcept;

    // optimise_route: the route deviates from every enemy unit of threat level 2 or more. Returns the
    // waypoints moved.
    std::size_t optimise_route() noexcept;

    [[nodiscard]] const icd::MsnStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::MsnPlan& plan() const noexcept { return plan_; }
    // True once per plan publication due: the caller then encodes plan() into the block.
    [[nodiscard]] bool take_plan_update() noexcept;
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] icd::MissionPhase phase() const noexcept { return phase_; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] const RouteManager& route() const noexcept { return route_; }
    [[nodiscard]] const Targeting& targeting() const noexcept { return targeting_; }
    [[nodiscard]] const OrderOfBattle& oob() const noexcept { return oob_; }
    [[nodiscard]] bool position_valid() const noexcept { return position_valid_; }

private:
    void fill_status() noexcept;
    void fill_plan() noexcept;
    void changed() noexcept { plan_dirty_ = true; }

    icd::MsnStatus status_{};
    icd::MsnPlan plan_{};
    RouteManager route_{};
    Targeting targeting_{};
    OrderOfBattle oob_{};
    icd::MissionPhase phase_{icd::MissionPhase::PLANNING};
    icd::CmdStatus cmd_status_{icd::CmdStatus::NONE};
    bool position_valid_{false};
    bool have_flight_{false};
    std::uint32_t last_flight_frame_{0};
    std::uint32_t frame_{0};
    bool plan_dirty_{true};
    bool plan_ready_{false};
    std::uint32_t last_plan_frame_{0};
    bool plan_published_once_{false};
    std::uint8_t plan_seq_{0};
    std::uint32_t self_test_started_{0};
    bool self_test_running_{false};
    bus::BitWord bit_{};
    bus::CommandSequence mode_seq_{};
    bus::CommandSequence cmd_seq_{};
};

}  // namespace fmofp::mission

#endif
