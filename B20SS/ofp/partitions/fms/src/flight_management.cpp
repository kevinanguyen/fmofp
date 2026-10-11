#include <fmofp/fms/flight_management.hpp>

#include <cmath>

namespace fmofp::fms {

namespace {

using icd::CmdStatus;
using icd::FmsFlightMode;
using icd::Ssm;
using icd::TerminalState;

constexpr double kPi = 3.141592653589793;
constexpr double kRadToDeg = 180.0 / kPi;  // math.degrees multiplies by this constant
constexpr double kMetresPerFoot = 0.3048;
constexpr double kMetresPerSecondPerKnot = 0.51444;  // the Python's factor
constexpr double kMassKg = 15000.0;
constexpr double kGravity = 9.81;

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

// fmsControl.flight_profiles, in fms_flight_mode order.
constexpr FlightProfile kProfiles[] = {
    {30.0, 20.0, 2.5, 15.0, 0.7, 3.0},   // NORMAL
    {80.0, 60.0, 9.0, 25.0, 1.0, 20.0},  // COMBAT
    {45.0, 30.0, 5.0, 20.0, 0.6, 5.0},   // STEALTH
    {60.0, 45.0, 7.0, 22.0, 0.9, 15.0},  // TRAINING
    {90.0, 85.0, 9.5, 30.0, 1.0, 25.0},  // EMERGENCY
};

double clamp(double v, double low, double high) noexcept { return v < low ? low : (v > high ? high : v); }

}  // namespace

const FlightProfile& profile(FmsFlightMode mode) noexcept {
    switch (mode) {
        case FmsFlightMode::NORMAL:
            return kProfiles[0];
        case FmsFlightMode::COMBAT:
            return kProfiles[1];
        case FmsFlightMode::STEALTH:
            return kProfiles[2];
        case FmsFlightMode::TRAINING:
            return kProfiles[3];
        case FmsFlightMode::EMERGENCY:
            return kProfiles[4];
    }
    return kProfiles[0];
}

TacticalSystems tactical_systems(FmsFlightMode mode) noexcept {
    TacticalSystems t;
    switch (mode) {
        case FmsFlightMode::COMBAT:
            t.countermeasures = icd::CountermeasuresState::READY;
            t.targeting = icd::TargetingState::ACTIVE;
            t.weapons = icd::WeaponsState::ARMED;
            t.stealth_mode = false;
            break;
        case FmsFlightMode::STEALTH:
            t.countermeasures = icd::CountermeasuresState::PASSIVE;
            t.targeting = icd::TargetingState::PASSIVE;
            t.weapons = icd::WeaponsState::SAFE;
            t.stealth_mode = true;
            break;
        case FmsFlightMode::EMERGENCY:
            t.countermeasures = icd::CountermeasuresState::ACTIVE;
            t.targeting = icd::TargetingState::READY;
            t.weapons = icd::WeaponsState::READY;
            t.stealth_mode = false;
            break;
        case FmsFlightMode::NORMAL:
        case FmsFlightMode::TRAINING:
            break;  // STANDBY, STANDBY, SAFE, off
    }
    return t;
}

LimitedAttitude limit_attitude(const AttitudeRequest& r, const Attitude& current, const FlightProfile& p) noexcept {
    LimitedAttitude out;
    Attitude& a = out.applied;
    a.roll = r.roll_present ? r.value.roll : current.roll;
    a.pitch = r.pitch_present ? r.value.pitch : current.pitch;
    a.yaw = r.yaw_present ? r.value.yaw : current.yaw;
    a.roll_rate = r.roll_rate_present ? r.value.roll_rate : current.roll_rate;
    a.pitch_rate = r.pitch_rate_present ? r.value.pitch_rate : current.pitch_rate;
    a.yaw_rate = r.yaw_rate_present ? r.value.yaw_rate : current.yaw_rate;

    if (std::fabs(a.roll) > p.max_bank_angle) {
        a.roll = std::copysign(p.max_bank_angle, a.roll);
        out.bank_limited = true;
    }
    if (std::fabs(a.pitch) > p.max_pitch_angle) {
        a.pitch = std::copysign(p.max_pitch_angle, a.pitch);
        out.pitch_limited = true;
    }
    if (std::fabs(a.roll_rate) > p.turn_rate_limit * 3.0) {
        a.roll_rate = std::copysign(p.turn_rate_limit * 3.0, a.roll_rate);
        out.roll_rate_limited = true;
    }
    if (std::fabs(a.pitch_rate) > p.turn_rate_limit * 2.0) {
        a.pitch_rate = std::copysign(p.turn_rate_limit * 2.0, a.pitch_rate);
        out.pitch_rate_limited = true;
    }
    if (std::fabs(a.yaw_rate) > p.turn_rate_limit) {
        a.yaw_rate = std::copysign(p.turn_rate_limit, a.yaw_rate);
        out.yaw_rate_limited = true;
    }
    return out;
}

double apply_thrust(ThrustState& state, double level) noexcept {
    const double min_thrust = 0.1;
    const double max_thrust = state.afterburner ? 1.0 : 0.9;
    const double applied = clamp(level, min_thrust, max_thrust);
    state.throttle_position = static_cast<std::uint8_t>(applied * 100.0);  // int() truncates
    if (applied > 0.9) {
        state.afterburner = true;
    } else if (applied < 0.85 && state.afterburner) {
        state.afterburner = false;
    }
    state.thrust = applied;
    return applied;
}

void apply_autopilot_targets(AutopilotTargets& t, const icd::FmsNavUpdateCmd& cmd) noexcept {
    if (cmd.altitude_present) {
        t.altitude_ft = clamp(cmd.altitude, 0.0, 60000.0);
    }
    if (cmd.airspeed_present) {
        t.airspeed_kt = clamp(cmd.airspeed, 100.0, 1000.0);
    }
    if (cmd.heading_present) {
        double h = std::fmod(cmd.heading, 360.0);  // Python's % is never negative
        if (h < 0.0) {
            h += 360.0;
        }
        t.heading_deg = h;
    }
    if (cmd.vertical_speed_present) {
        t.vertical_speed_fpm = clamp(cmd.vertical_speed, -8000.0, 6000.0);
    }
}

CmdStatus maneuver_request(icd::Maneuver maneuver, double dive_angle_deg, const FlightProfile& p,
                           AttitudeRequest& out) noexcept {
    out = AttitudeRequest{};
    switch (maneuver) {
        case icd::Maneuver::BREAK_RIGHT:
            out.roll_present = out.roll_rate_present = out.yaw_rate_present = true;
            out.value.roll = 60.0;
            out.value.roll_rate = 40.0;
            out.value.yaw_rate = 10.0;
            return CmdStatus::COMPLETED;
        case icd::Maneuver::BREAK_LEFT:
            out.roll_present = out.roll_rate_present = out.yaw_rate_present = true;
            out.value.roll = -60.0;
            out.value.roll_rate = -40.0;
            out.value.yaw_rate = -10.0;
            return CmdStatus::COMPLETED;
        case icd::Maneuver::BARREL_ROLL:
            return CmdStatus::NOT_IMPLEMENTED;  // "not implemented in this version"
        case icd::Maneuver::DEFENSIVE_SPLIT_S:
            out.roll_present = out.pitch_present = true;
            out.value.roll = 180.0;
            out.value.pitch = -80.0;
            return CmdStatus::COMPLETED;
        case icd::Maneuver::MAXIMUM_CLIMB:
            out.pitch_present = true;
            out.value.pitch = p.max_pitch_angle;
            return CmdStatus::COMPLETED;
        case icd::Maneuver::DIVE:
            out.pitch_present = true;
            out.value.pitch = dive_angle_deg == 0.0 ? -30.0 : dive_angle_deg;  // 0 means the default
            return CmdStatus::COMPLETED;
        case icd::Maneuver::NONE:
            return CmdStatus::REJECTED_INVALID;
    }
    return CmdStatus::REJECTED_INVALID;
}

double mach_number(double airspeed_kt) noexcept { return airspeed_kt / (661.4788 * 0.98); }

double energy_state(double airspeed_kt, double altitude_ft) noexcept {
    const double speed_ms = airspeed_kt * kMetresPerSecondPerKnot;
    const double altitude_m = altitude_ft * kMetresPerFoot;
    const double kinetic = 0.5 * kMassKg * (speed_ms * speed_ms);
    const double potential = kMassKg * kGravity * altitude_m;
    const double total = kinetic + potential;
    const double state = total / 1e10 * 100.0;
    return state < 100.0 ? state : 100.0;
}

EnergyMetrics energy_maneuverability(double altitude_ft, double airspeed_kt, double aoa_deg,
                                     const FlightProfile& p) noexcept {
    EnergyMetrics m;
    const double altitude_m = altitude_ft * kMetresPerFoot;
    const double speed_ms = airspeed_kt * kMetresPerSecondPerKnot;
    const double drag_coefficient = 0.02 + (std::fabs(aoa_deg) / 100.0);
    const double thrust_available = p.engine_power * 150000.0;
    const double air_density = 1.225 * std::exp(-altitude_m / 10000.0);
    const double drag = 0.5 * air_density * speed_ms * speed_ms * drag_coefficient * 30.0;
    const double weight = kMassKg * kGravity;
    m.specific_excess_power = speed_ms * (thrust_available - drag) / weight;
    const double thrust_weight_ratio = thrust_available / weight;
    const double v = speed_ms > 1.0 ? speed_ms : 1.0;
    if (thrust_weight_ratio > 1.0) {
        m.max_sustained_turn_rate =
            (kGravity * std::sqrt(thrust_weight_ratio * thrust_weight_ratio - 1.0) / v) * kRadToDeg;
    }
    const double g_force = p.max_g_force > 1.01 ? p.max_g_force : 1.01;
    m.max_instantaneous_turn_rate = (kGravity * std::sqrt(g_force * g_force - 1.0) / v) * kRadToDeg;
    m.max_climb_rate_fpm = m.specific_excess_power * 196.85;
    m.energy_advantage = m.specific_excess_power * 10.0;
    return m;
}

Status FlightManagement::init() noexcept {
    status_ = icd::FmsStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    status_.flight_mode = FmsFlightMode::NORMAL;
    flight_data_ = icd::FmsFlightData{};
    attitude_status_ = icd::FmsAttitudeStatus{};
    attitude_status_.roll_ssm = Ssm::NO_COMPUTED_DATA;  // nothing commanded yet
    attitude_status_.pitch_ssm = Ssm::NO_COMPUTED_DATA;
    attitude_status_.yaw_ssm = Ssm::NO_COMPUTED_DATA;
    nav_update_status_ = icd::FmsNavUpdateStatus{};
    maneuver_status_ = icd::FmsManeuverStatus{};
    control_status_ = icd::FmsControlStatus{};
    energy_ = icd::FmsEnergy{};
    sensors_ = icd::PlantSensors{};
    attitude_ = icd::FcsAttitude{};
    nav_ = icd::NavSolution{};
    have_sensors_ = have_attitude_ = have_nav_ = false;
    sensor_frame_ = attitude_frame_ = nav_frame_ = 0;
    sensors_fresh_ = attitude_fresh_ = nav_fresh_ = false;
    thrust_ = ThrustState{};
    targets_ = AutopilotTargets{};
    gear_ = icd::GearPosition::UP;
    mode_seq_ = bus::CommandSequence{};
    flight_mode_seq_ = bus::CommandSequence{};
    attitude_seq_ = bus::CommandSequence{};
    nav_update_seq_ = bus::CommandSequence{};
    maneuver_seq_ = bus::CommandSequence{};
    control_seq_ = bus::CommandSequence{};
    bit_ = bus::BitWord{};
    self_test_running_ = false;
    self_test_started_ = 0;
    produce_outputs(0);
    return Status::kOk;
}

void FlightManagement::reset() noexcept { (void)init(); }

void FlightManagement::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void FlightManagement::step(const icd::FmsSensors* sensors, const icd::FmsAttitudeIn* attitude,
                            const icd::FmsNavSolution* nav, std::uint32_t frame) noexcept {
    if (sensors != nullptr) {
        sensors_ = sensors->payload;
        have_sensors_ = true;
        sensor_frame_ = frame;
    }
    if (attitude != nullptr) {
        attitude_ = attitude->payload;
        have_attitude_ = true;
        attitude_frame_ = frame;
    }
    if (nav != nullptr) {
        nav_ = nav->payload;
        have_nav_ = true;
        nav_frame_ = frame;
    }
    sensors_fresh_ = have_sensors_ && frame - sensor_frame_ < kSensorStaleFrames;
    attitude_fresh_ = have_attitude_ && frame - attitude_frame_ < kSensorStaleFrames;
    nav_fresh_ = have_nav_ && frame - nav_frame_ < kNavStaleFrames;

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
    if (status_.state == TerminalState::INITIALIZING && sensors_fresh_) {
        status_.state = TerminalState::NORMAL;
    } else if (status_.state == TerminalState::NORMAL && !sensors_fresh_) {
        status_.state = TerminalState::DEGRADED;
    } else if (status_.state == TerminalState::DEGRADED && sensors_fresh_) {
        status_.state = TerminalState::NORMAL;
    }
    produce_outputs(frame);
}

Attitude FlightManagement::current_attitude() const noexcept {
    Attitude a;
    if (attitude_fresh_) {
        a.roll = usable(attitude_.roll_ssm) ? attitude_.roll : 0.0;
        a.pitch = usable(attitude_.pitch_ssm) ? attitude_.pitch : 0.0;
        a.yaw = usable(attitude_.yaw_ssm) ? attitude_.yaw : 0.0;
    }
    return a;
}

void FlightManagement::hold_attitude(const AttitudeRequest& request) noexcept {
    const LimitedAttitude l = limit_attitude(request, current_attitude(), profile(status_.flight_mode));
    attitude_status_.roll_ssm = Ssm::NORMAL;
    attitude_status_.pitch_ssm = Ssm::NORMAL;
    attitude_status_.yaw_ssm = Ssm::NORMAL;
    attitude_status_.roll = l.applied.roll;
    attitude_status_.pitch = l.applied.pitch;
    attitude_status_.yaw = l.applied.yaw;
    attitude_status_.roll_rate = l.applied.roll_rate;
    attitude_status_.pitch_rate = l.applied.pitch_rate;
    attitude_status_.yaw_rate = l.applied.yaw_rate;
    attitude_status_.bank_limited = l.bank_limited;
    attitude_status_.pitch_limited = l.pitch_limited;
    attitude_status_.roll_rate_limited = l.roll_rate_limited;
    attitude_status_.pitch_rate_limited = l.pitch_rate_limited;
    attitude_status_.yaw_rate_limited = l.yaw_rate_limited;
}

void FlightManagement::command_mode(const icd::FmsModeCmd& cmd) noexcept {
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
}

void FlightManagement::command_flight_mode(const icd::FmsFlightModeCmd& cmd) noexcept {
    if (!flight_mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.flight_mode_cmd_seq = cmd.cmd_seq;
    if (!icd::is_valid(cmd.flight_mode)) {
        status_.flight_mode_status = CmdStatus::REJECTED_INVALID;
        return;
    }
    status_.flight_mode = cmd.flight_mode;
    status_.flight_mode_status = CmdStatus::COMPLETED;
}

void FlightManagement::command_attitude(const icd::FmsAttitudeCmd& cmd) noexcept {
    if (!attitude_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    attitude_status_.last_cmd_seq = cmd.cmd_seq;
    AttitudeRequest r;
    r.roll_present = cmd.roll_present;
    r.pitch_present = cmd.pitch_present;
    r.yaw_present = cmd.yaw_present;
    r.roll_rate_present = cmd.roll_rate_present;
    r.pitch_rate_present = cmd.pitch_rate_present;
    r.yaw_rate_present = cmd.yaw_rate_present;
    r.value = Attitude{cmd.roll, cmd.pitch, cmd.yaw, cmd.roll_rate, cmd.pitch_rate, cmd.yaw_rate};
    if (!std::isfinite(cmd.roll) || !std::isfinite(cmd.pitch) || !std::isfinite(cmd.yaw) ||
        !std::isfinite(cmd.roll_rate) || !std::isfinite(cmd.pitch_rate) || !std::isfinite(cmd.yaw_rate)) {
        attitude_status_.status = CmdStatus::REJECTED_INVALID;
        return;
    }
    hold_attitude(r);
    attitude_status_.status = CmdStatus::COMPLETED;
}

void FlightManagement::command_nav_update(const icd::FmsNavUpdateCmd& cmd) noexcept {
    if (!nav_update_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    nav_update_status_.last_cmd_seq = cmd.cmd_seq;
    if (!std::isfinite(cmd.altitude) || !std::isfinite(cmd.airspeed) || !std::isfinite(cmd.heading) ||
        !std::isfinite(cmd.vertical_speed)) {
        nav_update_status_.status = CmdStatus::REJECTED_INVALID;
        return;
    }
    apply_autopilot_targets(targets_, cmd);
    nav_update_status_.status = CmdStatus::COMPLETED;
}

void FlightManagement::command_maneuver(const icd::FmsManeuverCmd& cmd) noexcept {
    if (!maneuver_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    maneuver_status_.last_cmd_seq = cmd.cmd_seq;
    maneuver_status_.maneuver = cmd.maneuver;
    AttitudeRequest r;
    const CmdStatus s = std::isfinite(cmd.dive_angle)
                            ? maneuver_request(cmd.maneuver, cmd.dive_angle, profile(status_.flight_mode), r)
                            : CmdStatus::REJECTED_INVALID;
    if (s == CmdStatus::COMPLETED) {
        hold_attitude(r);
    }
    maneuver_status_.status = s;
}

void FlightManagement::command_control(const icd::FmsControlCmd& cmd) noexcept {
    if (!control_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    control_status_.last_cmd_seq = cmd.cmd_seq;
    const bool gear_ok = cmd.gear == icd::GearPosition::UP || cmd.gear == icd::GearPosition::DOWN;
    if ((!cmd.thrust_present && !cmd.gear_present) || (cmd.gear_present && !gear_ok) ||
        (cmd.thrust_present && !std::isfinite(cmd.thrust))) {
        control_status_.status = CmdStatus::REJECTED_INVALID;
        return;
    }
    if (cmd.thrust_present) {
        (void)apply_thrust(thrust_, cmd.thrust);
    }
    if (cmd.gear_present) {
        gear_ = cmd.gear;
    }
    control_status_.status = CmdStatus::COMPLETED;
}

void FlightManagement::produce_outputs(std::uint32_t frame) noexcept {
    (void)frame;
    // Status: the terminal part, then what fmsControl's mode set.
    status_.healthy = !bit_.failed;
    status_.mode_transition = false;
    const TacticalSystems t = tactical_systems(status_.flight_mode);
    status_.countermeasures = t.countermeasures;
    status_.targeting = t.targeting;
    status_.weapons = t.weapons;
    status_.stealth_mode = t.stealth_mode;

    // Autopilot targets and thrust/gear in force.
    nav_update_status_.altitude = targets_.altitude_ft;
    nav_update_status_.airspeed = targets_.airspeed_kt;
    nav_update_status_.heading = targets_.heading_deg;
    nav_update_status_.vertical_speed = targets_.vertical_speed_fpm;
    control_status_.thrust = thrust_.thrust;
    control_status_.throttle_position = thrust_.throttle_position;
    control_status_.afterburner = thrust_.afterburner;
    control_status_.gear = gear_;

    // Flight data: what get_flight_data() handed the displays.
    icd::FmsFlightData& d = flight_data_;
    d = icd::FmsFlightData{};
    d.flight_mode = status_.flight_mode;
    const auto carry = [](bool fresh, Ssm ssm, double value, Ssm& out_ssm, double& out) noexcept {
        const bool good = fresh && usable(ssm);
        out_ssm = good ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
        out = good ? value : 0.0;
        return good;
    };
    carry(attitude_fresh_, attitude_.roll_ssm, attitude_.roll, d.roll_ssm, d.roll);
    carry(attitude_fresh_, attitude_.pitch_ssm, attitude_.pitch, d.pitch_ssm, d.pitch);
    carry(sensors_fresh_, sensors_.heading_ssm, sensors_.heading, d.heading_ssm, d.heading);
    const bool airspeed_ok = carry(sensors_fresh_, sensors_.airspeed_ssm, sensors_.airspeed, d.airspeed_ssm, d.airspeed);
    carry(sensors_fresh_, sensors_.altitude_ssm, sensors_.altitude, d.altitude_ssm, d.altitude);
    carry(sensors_fresh_, sensors_.vertical_speed_ssm, sensors_.vertical_speed, d.vertical_speed_ssm,
          d.vertical_speed);
    carry(sensors_fresh_, sensors_.g_ssm, sensors_.g_force, d.g_ssm, d.g_force);
    const bool aoa_ok = carry(sensors_fresh_, sensors_.aoa_ssm, sensors_.angle_of_attack, d.aoa_ssm, d.angle_of_attack);
    // Mach from the airspeed alone, as the Python did; ground speed is the
    // airspeed, the plant having no wind.
    carry(airspeed_ok, Ssm::NORMAL, mach_number(sensors_.airspeed), d.mach_ssm, d.mach);
    carry(airspeed_ok, Ssm::NORMAL, sensors_.airspeed, d.ground_speed_ssm, d.ground_speed);
    d.track_ssm = Ssm::NO_COMPUTED_DATA;     // never computed by the Python
    d.sideslip_ssm = Ssm::NO_COMPUTED_DATA;  // no sensor for it
    const bool position_ok = nav_fresh_ && usable(nav_.position_ssm);
    d.position_ssm = position_ok ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    d.latitude = position_ok ? nav_.latitude : 0.0;
    d.longitude = position_ok ? nav_.longitude : 0.0;
    const bool nav_altitude_ok = carry(nav_fresh_, nav_.altitude_ssm, nav_.altitude, d.nav_altitude_ssm, d.nav_altitude);
    carry(airspeed_ok && nav_altitude_ok, Ssm::NORMAL, energy_state(sensors_.airspeed, nav_.altitude), d.energy_ssm,
          d.energy_state);

    // Energy maneuverability from the same inputs and the profile in force.
    energy_ = icd::FmsEnergy{};
    if (airspeed_ok && nav_altitude_ok && aoa_ok) {
        const EnergyMetrics m =
            energy_maneuverability(nav_.altitude, sensors_.airspeed, sensors_.angle_of_attack, profile(status_.flight_mode));
        energy_.ssm = Ssm::NORMAL;
        energy_.specific_excess_power = m.specific_excess_power;
        energy_.max_sustained_turn_rate = m.max_sustained_turn_rate;
        energy_.max_instantaneous_turn_rate = m.max_instantaneous_turn_rate;
        energy_.max_climb_rate = m.max_climb_rate_fpm;
        energy_.energy_advantage = m.energy_advantage;
    } else {
        energy_.ssm = Ssm::NO_COMPUTED_DATA;
    }
}

}  // namespace fmofp::fms
