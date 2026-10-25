#include <fmofp/fcs/flight_control.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::fcs {

namespace {

using icd::Ssm;
using icd::TerminalState;

bool usable(Ssm ssm) noexcept { return ssm == Ssm::NORMAL; }

// The Python read missing values through dict.get() with these defaults, so
// an invalid sensor behaves exactly like a missing one did.
constexpr double kDefaultAltitudeFt = 30000.0;
constexpr double kDefaultVsFpm = 0.0;

// A state the FCS can be commanded into from the bus.
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

// Surfaces in FCS_SURFACE_STATUS word order; ControlSurface::NONE has no slot.
double* surface_slot(icd::FcsSurfaceStatus& s, icd::ControlSurface surface) noexcept {
    switch (surface) {
        case icd::ControlSurface::AILERONS:
            return &s.ailerons;
        case icd::ControlSurface::ELEVATOR:
            return &s.elevator;
        case icd::ControlSurface::RUDDER:
            return &s.rudder;
        case icd::ControlSurface::FLAPS:
            return &s.flaps;
        case icd::ControlSurface::SPOILERS:
            return &s.spoilers;
        case icd::ControlSurface::NONE:
            return nullptr;
    }
    return nullptr;
}

// Self test takes a fixed number of frames so TERM_STATUS shows it in progress.
constexpr std::uint32_t kSelfTestFrames = 10;

}  // namespace

Status FlightControl::init() noexcept {
    status_ = icd::FcsStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    attitude_ = icd::FcsAttitude{};
    gcas_ = icd::FcsGcasAlerts{};
    perf_ = icd::FcsPerfStatus{};
    surfaces_ = icd::FcsSurfaceStatus{};
    // Surfaces start at neutral and are known, as fmsControl's initial values are.
    surfaces_.ailerons_ssm = Ssm::NORMAL;
    surfaces_.elevator_ssm = Ssm::NORMAL;
    surfaces_.rudder_ssm = Ssm::NORMAL;
    surfaces_.flaps_ssm = Ssm::NORMAL;
    surfaces_.spoilers_ssm = Ssm::NORMAL;
    last_sensors_ = icd::PlantSensors{};
    have_sensors_ = false;
    last_sensor_frame_ = 0;
    last_terrain_ = icd::RdrTfrGuidance{};
    have_terrain_ = false;
    last_terrain_frame_ = 0;
    last_engine_ = icd::EngEngineStatus{};
    have_engine_ = false;
    last_engine_frame_ = 0;
    self_test_running_ = false;
    self_test_started_ = 0;
    bit_ = bus::BitWord{};
    mode_seq_ = bus::CommandSequence{};
    surface_seq_ = bus::CommandSequence{};
    return Status::kOk;
}

void FlightControl::reset() noexcept { (void)init(); }

void FlightControl::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void FlightControl::step(const icd::FcsSensors* sensors, const icd::FcsTerrain* terrain,
                         const icd::BcastEngEngineStatus* engine, std::uint32_t frame) noexcept {
    if (sensors != nullptr) {
        last_sensors_ = sensors->payload;
        have_sensors_ = true;
        last_sensor_frame_ = frame;
    }
    if (terrain != nullptr) {
        last_terrain_ = terrain->payload;
        have_terrain_ = true;
        last_terrain_frame_ = frame;
    }
    if (engine != nullptr) {
        last_engine_ = engine->payload;
        have_engine_ = true;
        last_engine_frame_ = frame;
    }
    const bool fresh = have_sensors_ && frame - last_sensor_frame_ < kSensorStaleFrames;
    const bool terrain_fresh = have_terrain_ && frame - last_terrain_frame_ < kTerrainStaleFrames;
    const bool engine_fresh = have_engine_ && frame - last_engine_frame_ < kEngineStaleFrames;

    if (self_test_running_) {
        if (self_test_started_ == 0) {
            self_test_started_ = frame == 0 ? 1 : frame;
        } else if (frame - self_test_started_ >= kSelfTestFrames) {
            self_test_running_ = false;
            self_test_started_ = 0;
            bit_.in_progress = false;  // nothing in this partition can fail yet
        }
    }

    // State: INITIALIZING until the first sensors; NORMAL while they keep
    // coming; DEGRADED when they stop. A commanded state (STANDBY, TEST, ...)
    // is kept until commanded again.
    if (status_.state == TerminalState::INITIALIZING && fresh) {
        status_.state = TerminalState::NORMAL;
    } else if (status_.state == TerminalState::NORMAL && !fresh) {
        status_.state = TerminalState::DEGRADED;
    } else if (status_.state == TerminalState::DEGRADED && fresh) {
        status_.state = TerminalState::NORMAL;
    }

    status_.healthy = !bit_.failed;
    status_.mode_transition = false;
    if (status_.state == TerminalState::NORMAL || status_.state == TerminalState::DEGRADED) {
        produce_outputs(fresh, terrain_fresh, engine_fresh);
    } else {
        invalidate_outputs();
    }
}

void FlightControl::produce_outputs(bool sensors_valid, bool terrain_valid, bool engine_valid) noexcept {
    if (!sensors_valid) {
        invalidate_outputs();
        return;
    }
    const icd::PlantSensors& s = last_sensors_;
    TerrainClearance clearance{};
    clearance.valid = terrain_valid && usable(last_terrain_.clearance_ssm);
    clearance.feet = last_terrain_.min_clearance;
    // The total fuel flow counts only engines whose data is valid; with neither
    // valid the engine computer has no flow to offer.
    EngineFuelFlow fuel{};
    fuel.valid = engine_valid && (usable(last_engine_.engine_1_ssm) || usable(last_engine_.engine_2_ssm));
    fuel.kgh = last_engine_.total_fuel_flow;

    attitude_.roll_ssm = s.roll_ssm;
    attitude_.pitch_ssm = s.pitch_ssm;
    attitude_.yaw_ssm = s.heading_ssm;
    attitude_.roll = s.roll;
    attitude_.pitch = s.pitch;
    // FCS_ATTITUDE carries yaw as -180..+180; the plant publishes heading 0..360.
    attitude_.yaw = s.heading > 180.0 ? s.heading - 360.0 : s.heading;

    gcas_ = evaluate_gcas(s, clearance, gcas_thresholds_);
    perf_ = evaluate_envelope(s, fuel, limits_);
}

void FlightControl::invalidate_outputs() noexcept {
    attitude_.roll_ssm = Ssm::NO_COMPUTED_DATA;
    attitude_.pitch_ssm = Ssm::NO_COMPUTED_DATA;
    attitude_.yaw_ssm = Ssm::NO_COMPUTED_DATA;
    gcas_ = icd::FcsGcasAlerts{};  // status OFF, no alerts, every SSM FAILURE_WARNING
    gcas_.vertical_speed_ssm = Ssm::NO_COMPUTED_DATA;
    gcas_.altitude_ssm = Ssm::NO_COMPUTED_DATA;
    gcas_.clearance_ssm = Ssm::NO_COMPUTED_DATA;
    perf_ = icd::FcsPerfStatus{};
    perf_.g_ssm = Ssm::NO_COMPUTED_DATA;
    perf_.aoa_ssm = Ssm::NO_COMPUTED_DATA;
    perf_.fuel_ssm = Ssm::NO_COMPUTED_DATA;
}

icd::FcsGcasAlerts FlightControl::evaluate_gcas(const icd::PlantSensors& s, const TerrainClearance& clearance,
                                                const GcasThresholds& t) noexcept {
    icd::FcsGcasAlerts a{};
    const bool alt_ok = usable(s.altitude_ssm);
    const bool vs_ok = usable(s.vertical_speed_ssm);
    const bool clearance_ok = clearance.valid && std::isfinite(clearance.feet);
    const double alt = alt_ok ? s.altitude : kDefaultAltitudeFt;
    const double vs = vs_ok ? s.vertical_speed : kDefaultVsFpm;

    // groundCollisionAvoidanceSys._evaluate, decision for decision.
    if (vs <= t.pullup_vs_fpm && alt < t.pullup_alt_ft) {
        a.pullup = true;
    } else if (vs <= t.sink_vs_fpm) {
        a.sink_rate = true;
    }
    if (clearance_ok && clearance.feet < t.terrain_ft) {
        a.terrain = true;
    }
    if (alt < t.alt_low_ft && !a.terrain) {
        a.alt_low = true;
    }

    const bool warning = a.pullup || a.sink_rate || a.terrain;
    a.status = warning ? icd::GcasStatus::WARNING : a.alt_low ? icd::GcasStatus::CAUTION : icd::GcasStatus::ARMED;

    a.vertical_speed_ssm = vs_ok ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    a.altitude_ssm = alt_ok ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    a.clearance_ssm = clearance_ok ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    a.vertical_speed = vs_ok ? s.vertical_speed : 0.0;
    a.altitude = alt_ok ? s.altitude : 0.0;
    a.terrain_clearance = clearance_ok ? clearance.feet : 0.0;
    return a;
}

icd::FcsPerfStatus FlightControl::evaluate_envelope(const icd::PlantSensors& s, const EngineFuelFlow& fuel,
                                                    const EnvelopeLimits& l) noexcept {
    icd::FcsPerfStatus p{};
    // performaneMonitoring._check_envelope: abs(state.get(param, 0)) > limit,
    // so an invalid value reads as 0 and cannot exceed anything.
    const double g = usable(s.g_ssm) ? s.g_force : 0.0;
    const double aoa = usable(s.aoa_ssm) ? s.angle_of_attack : 0.0;
    const double roll = usable(s.roll_ssm) ? s.roll : 0.0;
    const double pitch = usable(s.pitch_ssm) ? s.pitch : 0.0;
    const double vs = usable(s.vertical_speed_ssm) ? s.vertical_speed : 0.0;
    p.exceed_g = std::fabs(g) > l.g;
    p.exceed_aoa = std::fabs(aoa) > l.aoa_deg;
    p.exceed_roll = std::fabs(roll) > l.roll_deg;
    p.exceed_pitch = std::fabs(pitch) > l.pitch_deg;
    p.exceed_vertical_speed = std::fabs(vs) > l.vs_fpm;
    p.g_ssm = usable(s.g_ssm) ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    p.aoa_ssm = usable(s.aoa_ssm) ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    p.g_force = usable(s.g_ssm) ? s.g_force : 0.0;
    p.angle_of_attack = usable(s.aoa_ssm) ? s.angle_of_attack : 0.0;
    // performaneMonitoring._compute_fuel_efficiency: speed_kts / (ff_kgh / 60),
    // 0.0 when the flow is not positive; the flow is the engine computer's
    // total over the valid engines (ICD README section 2).
    const bool fuel_ok = fuel.valid && std::isfinite(fuel.kgh) && usable(s.airspeed_ssm) && std::isfinite(s.airspeed);
    p.fuel_ssm = fuel_ok ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    p.fuel_efficiency = 0.0;
    if (fuel_ok && fuel.kgh > 0.0) {
        p.fuel_efficiency = std::min(kMaxFuelEfficiency, std::max(0.0, s.airspeed / (fuel.kgh / 60.0)));
    }
    return p;
}

void FlightControl::command_mode(const icd::FcsModeCmd& cmd) noexcept {
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

void FlightControl::command_surface(const icd::FcsSurfaceCmd& cmd) noexcept {
    if (!surface_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    surfaces_.last_cmd_seq = cmd.cmd_seq;
    double* slot = surface_slot(surfaces_, cmd.surface);
    if (slot == nullptr || !std::isfinite(cmd.position)) {
        surfaces_.last_cmd_status = icd::CmdStatus::REJECTED_INVALID;
        return;
    }
    // Positions are percent of travel; fmsControl.set_control_surfaces clamps
    // ailerons/elevator/rudder to +-100, flaps and spoilers to 0..100.
    const bool signed_surface = cmd.surface == icd::ControlSurface::AILERONS ||
                                cmd.surface == icd::ControlSurface::ELEVATOR ||
                                cmd.surface == icd::ControlSurface::RUDDER;
    const double low = signed_surface ? -100.0 : 0.0;
    const double high = 100.0;
    if (cmd.position < low || cmd.position > high) {
        *slot = cmd.position < low ? low : high;
        surfaces_.last_cmd_status = icd::CmdStatus::REJECTED_INVALID;
        return;
    }
    *slot = cmd.position;
    surfaces_.last_cmd_status = icd::CmdStatus::COMPLETED;
}

}  // namespace fmofp::fcs
