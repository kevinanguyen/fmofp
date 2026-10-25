#include <fmofp/engine/engine_management.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace fmofp::engine {

namespace {

using icd::Ssm;
using icd::TerminalState;

bool usable(Ssm ssm) noexcept { return ssm == Ssm::NORMAL; }

double clamp_pct(double pct) noexcept { return pct < 0.0 ? 0.0 : pct > 100.0 ? 100.0 : pct; }

// A state the engine computer can be commanded into from the bus (as the FCS).
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

// fuel_tank enum value to tank index, or kTanks for NONE.
std::size_t tank_index(icd::FuelTank tank) noexcept {
    switch (tank) {
        case icd::FuelTank::MAIN_LEFT:
            return 0;
        case icd::FuelTank::MAIN_RIGHT:
            return 1;
        case icd::FuelTank::CENTER:
            return 2;
        case icd::FuelTank::NONE:
            return kTanks;
    }
    return kTanks;
}

double line_weights(const std::array<FuelLine, 2>& lines) noexcept {
    return std::accumulate(lines.begin(), lines.end(), 0.0,
                           [](double acc, const FuelLine& line) { return acc + line.weight; });
}

constexpr std::uint32_t kSelfTestFrames = 10;
constexpr double kSecondsPerHour = 3600.0;
constexpr double kMaxReportedKg = 65535.0;

}  // namespace

// ---- ThrustManager ---------------------------------------------------------------------

void ThrustManager::set_thrust(std::size_t engine, double pct) noexcept {
    if (engine < kEngines) {
        thrust[engine] = clamp_pct(pct);
    }
}

void ThrustManager::set_all(double pct) noexcept { thrust.fill(clamp_pct(pct)); }

void ThrustManager::engage(double target) noexcept {
    autothrottle = true;
    target_kt = target;
    last_error = 0.0;
    integral = 0.0;
}

void ThrustManager::disengage() noexcept {
    autothrottle = false;
    target_kt = 0.0;
}

void ThrustManager::update_autothrottle(double airspeed_kt, double dt) noexcept {
    if (!autothrottle || !(dt > 0.0)) {
        return;
    }
    const double error = target_kt - airspeed_kt;
    integral = std::max(-kAutothrottleIntegralLimit, std::min(kAutothrottleIntegralLimit, integral + error * dt));
    const double delta_pct = kAutothrottleKp * error + kAutothrottleKi * integral;
    last_error = error;
    const double average = std::accumulate(thrust.begin(), thrust.end(), 0.0) / static_cast<double>(kEngines);
    set_all(average + delta_pct);
}

double ThrustManager::asymmetry() const noexcept {
    const auto [lo, hi] = std::minmax_element(thrust.begin(), thrust.end());
    return *hi - *lo;
}

// ---- FuelSystem ------------------------------------------------------------------------

void FuelSystem::init() noexcept {
    levels_ = kTankCapacityKg;  // every tank full, as add_tank left them
    last_levels_ = {};
    rates_kg_s_ = {};
    expected_burn_ = {};
    transfers_out_ = {};
    flags_ = {};
    transfer_left_ = false;
    transfer_right_ = false;
    have_last_ = false;
    transferred_total_ = 0.0;
}

void FuelSystem::step(const std::array<double, kEngines>& fuel_flow_kgh, double dt) noexcept {
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        return;
    }
    burn(fuel_flow_kgh, dt);
    policy();
    poll(dt);
}

void FuelSystem::burn(const std::array<double, kEngines>& fuel_flow_kgh, double dt) noexcept {
    // FuelSystem.update: each engine draws from every tank on its lines. The
    // Python drew its (never set) fuel_consumption_rate from each tank in
    // full; this splits the measured flow by the lines' weights.
    for (std::size_t e = 0; e < kEngines; ++e) {
        const double flow = std::isfinite(fuel_flow_kgh[e]) ? std::max(0.0, fuel_flow_kgh[e]) : 0.0;
        const double kg = flow / kSecondsPerHour * dt;
        const double weights = line_weights(kFuelLines[e]);
        for (const FuelLine& line : kFuelLines[e]) {
            const double required = kg * line.weight / weights;
            const double consumed = std::min(levels_[line.tank], required);
            levels_[line.tank] -= consumed;
            expected_burn_[line.tank] += required;  // the monitor expects the rate's worth, as the Python did
        }
    }
}

void FuelSystem::policy() noexcept {
    // perform_fuel_transfer, with its one reading of the levels before both transfers.
    transfer_left_ = false;
    transfer_right_ = false;
    const std::array<double, kTanks> levels = levels_;
    if (levels[2] > kTransferSourceAboveKg) {
        if (levels[0] < kTransferFeedBelowKg) {
            transfer_left_ = transfer(2, 0, kTransferAmountKg) > 0.0;
        }
        if (levels[1] < kTransferFeedBelowKg) {
            transfer_right_ = transfer(2, 1, kTransferAmountKg) > 0.0;
        }
    }
}

double FuelSystem::transfer(std::size_t source, std::size_t destination, double kg) noexcept {
    if (source >= kTanks || destination >= kTanks || source == destination || !(kg > 0.0) || !std::isfinite(kg)) {
        return 0.0;
    }
    const double space = kTankCapacityKg[destination] - levels_[destination];
    const double moved = std::max(0.0, std::min({kg, levels_[source], space}));
    levels_[source] -= moved;
    levels_[destination] += moved;
    transfers_out_[source] += moved;
    transferred_total_ += moved;
    return moved;
}

void FuelSystem::set_level(std::size_t tank, double kg) noexcept {
    if (tank < kTanks && std::isfinite(kg)) {
        levels_[tank] = std::max(0.0, std::min(kTankCapacityKg[tank], kg));
    }
}

void FuelSystem::poll(double dt) noexcept {
    // FuelMonitor.poll: consumption rates from the drop since the last poll
    // (less the transfers, which are not consumption); a drop beyond what the
    // burn and the transfers explain, by more than 2% of the capacity, is an
    // unexplained loss; low and critical by percent of capacity.
    for (std::size_t tank = 0; tank < kTanks; ++tank) {
        const double capacity = kTankCapacityKg[tank];
        if (have_last_) {
            const double drop = last_levels_[tank] - levels_[tank];
            rates_kg_s_[tank] = std::max(0.0, (drop - transfers_out_[tank]) / dt);
            if (drop > expected_burn_[tank] + transfers_out_[tank] + capacity * kLeakUnexplainedDropFraction) {
                flags_[tank].leak = true;
            }
        }
        last_levels_[tank] = levels_[tank];
        expected_burn_[tank] = 0.0;
        transfers_out_[tank] = 0.0;
        const double pct = levels_[tank] / capacity * 100.0;
        flags_[tank].critical = pct <= kCriticalFuelPct;
        flags_[tank].low = !flags_[tank].critical && pct <= kLowFuelPct;
    }
    have_last_ = true;
}

double FuelSystem::total() const noexcept { return std::accumulate(levels_.begin(), levels_.end(), 0.0); }

double FuelSystem::burn_rate_kgh() const noexcept {
    return std::accumulate(rates_kg_s_.begin(), rates_kg_s_.end(), 0.0) * kSecondsPerHour;
}

// ---- EngineManagement ------------------------------------------------------------------

Status EngineManagement::init() noexcept {
    status_ = icd::EngStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    thrust_cmd_ = icd::EngThrustCmd{};
    engine_status_ = icd::EngEngineStatus{};
    fuel_status_ = icd::EngFuelStatus{};
    thrust_ = ThrustManager{};
    fuel_.init();
    commands_valid_ = false;
    last_engines_ = icd::PlantEngine{};
    have_engines_ = false;
    last_engine_frame_ = 0;
    airspeed_kt_ = 0.0;
    have_airspeed_ = false;
    last_flight_frame_ = 0;
    afterburner_ = false;
    have_throttle_ = false;
    last_throttle_ = 0;
    cmd_status_ = icd::CmdStatus::NONE;
    self_test_started_ = 0;
    self_test_running_ = false;
    bit_ = bus::BitWord{};
    mode_seq_ = bus::CommandSequence{};
    cmd_seq_ = bus::CommandSequence{};
    produce_outputs(false);
    return Status::kOk;
}

void EngineManagement::reset() noexcept { (void)init(); }

void EngineManagement::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void EngineManagement::step(const icd::EngFmsControl* fms, const icd::BcastFmsFlightData* flight,
                            const icd::EngEngineIn* engines, std::uint32_t frame) noexcept {
    if (fms != nullptr) {
        afterburner_ = fms->payload.afterburner;
        // The FMS throttle is the lever: a new position commands both engines
        // (set_all_thrust), unless the autothrottle owns the levers. After a
        // disengage the thrust stays where the autothrottle left it until the
        // lever moves.
        const std::uint8_t throttle = fms->payload.throttle_position;
        if (!have_throttle_ || throttle != last_throttle_) {
            have_throttle_ = true;
            last_throttle_ = throttle;
            if (!thrust_.autothrottle) {
                thrust_.set_all(static_cast<double>(throttle));
                commands_valid_ = true;
            }
        }
    }
    if (flight != nullptr) {
        have_airspeed_ = usable(flight->payload.airspeed_ssm) && std::isfinite(flight->payload.airspeed);
        airspeed_kt_ = flight->payload.airspeed;
        last_flight_frame_ = frame;
    }
    if (engines != nullptr) {
        last_engines_ = engines->payload;
        have_engines_ = true;
        last_engine_frame_ = frame;
    }
    const bool engines_fresh = have_engines_ && frame - last_engine_frame_ < kEngineStaleFrames;
    const bool airspeed_fresh = have_airspeed_ && frame - last_flight_frame_ < kFlightDataStaleFrames;

    if (self_test_running_) {
        if (self_test_started_ == 0) {
            self_test_started_ = frame == 0 ? 1 : frame;
        } else if (frame - self_test_started_ >= kSelfTestFrames) {
            self_test_running_ = false;
            self_test_started_ = 0;
            bit_.in_progress = false;
        }
    }

    // State: INITIALIZING until the first engine data; NORMAL while it keeps
    // coming; DEGRADED when it stops. A commanded state is kept until commanded again.
    if (status_.state == TerminalState::INITIALIZING && engines_fresh) {
        status_.state = TerminalState::NORMAL;
    } else if (status_.state == TerminalState::NORMAL && !engines_fresh) {
        status_.state = TerminalState::DEGRADED;
    } else if (status_.state == TerminalState::DEGRADED && engines_fresh) {
        status_.state = TerminalState::NORMAL;
    }

    // The autothrottle's PI step at the Python's 2 Hz, on a fresh airspeed.
    if (frame % kAutothrottleFrames == 0 && thrust_.autothrottle && airspeed_fresh) {
        thrust_.update_autothrottle(airspeed_kt_, kAutothrottleDt);
        commands_valid_ = true;
    }
    // The fuel pass at the Python's 10 Hz, burning the measured flow of each engine that reports one.
    if (frame % kFuelFrames == 0) {
        std::array<double, kEngines> flow{};
        if (engines_fresh) {
            flow[0] = usable(last_engines_.engine_1_ssm) ? last_engines_.fuel_flow_1 : 0.0;
            flow[1] = usable(last_engines_.engine_2_ssm) ? last_engines_.fuel_flow_2 : 0.0;
        }
        fuel_.step(flow, kFuelDt);
    }

    status_.healthy = !bit_.failed;
    status_.mode_transition = false;
    produce_outputs(engines_fresh);
}

void EngineManagement::produce_outputs(bool engines_fresh) noexcept {
    // The command to the plant's engines.
    thrust_cmd_.thrust_1 = thrust_.thrust[0];
    thrust_cmd_.thrust_2 = thrust_.thrust[1];
    thrust_cmd_.afterburner = afterburner_;
    thrust_cmd_.autothrottle_engaged = thrust_.autothrottle;
    thrust_cmd_.asymmetric_thrust = thrust_.asymmetric();
    thrust_cmd_.commands_valid = commands_valid_;
    thrust_cmd_.asymmetry = std::min(100.0, thrust_.asymmetry());

    // The engines as measured, with the assessment, or NO_COMPUTED_DATA.
    icd::EngEngineStatus e{};
    const icd::PlantEngine& p = last_engines_;
    const bool one = engines_fresh && usable(p.engine_1_ssm);
    const bool two = engines_fresh && usable(p.engine_2_ssm);
    e.engine_1_ssm = one ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    e.engine_2_ssm = two ? Ssm::NORMAL : Ssm::NO_COMPUTED_DATA;
    if (one) {
        e.thrust_1 = p.thrust_1;
        e.n1_1 = p.n1_1;
        e.n2_1 = p.n2_1;
        e.egt_1 = p.egt_1;
        e.fuel_flow_1 = p.fuel_flow_1;
        e.oil_pressure_1 = p.oil_pressure_1;
        e.oil_temperature_1 = p.oil_temperature_1;
        e.vibration_1 = p.vibration_1;
        e.compressor_efficiency_1 = p.compressor_efficiency_1;
        e.egt_high_1 = p.egt_1 > kEgtHighC;
        e.oil_low_1 = p.oil_pressure_1 < kOilLowPsi;
        e.oil_hot_1 = p.oil_temperature_1 > kOilHotC;
        e.vibration_high_1 = p.vibration_1 > kVibrationHigh;
        e.overspeed_1 = p.n1_1 >= kOverspeedPct || p.n2_1 >= kOverspeedPct;
    }
    if (two) {
        e.thrust_2 = p.thrust_2;
        e.n1_2 = p.n1_2;
        e.n2_2 = p.n2_2;
        e.egt_2 = p.egt_2;
        e.fuel_flow_2 = p.fuel_flow_2;
        e.oil_pressure_2 = p.oil_pressure_2;
        e.oil_temperature_2 = p.oil_temperature_2;
        e.vibration_2 = p.vibration_2;
        e.compressor_efficiency_2 = p.compressor_efficiency_2;
        e.egt_high_2 = p.egt_2 > kEgtHighC;
        e.oil_low_2 = p.oil_pressure_2 < kOilLowPsi;
        e.oil_hot_2 = p.oil_temperature_2 > kOilHotC;
        e.vibration_high_2 = p.vibration_2 > kVibrationHigh;
        e.overspeed_2 = p.n1_2 >= kOverspeedPct || p.n2_2 >= kOverspeedPct;
    }
    e.total_fuel_flow = (one ? p.fuel_flow_1 : 0.0) + (two ? p.fuel_flow_2 : 0.0);
    engine_status_ = e;

    // The tanks.
    const std::array<TankFlags, kTanks>& tank_flags = fuel_.flags();
    icd::EngFuelStatus f{};
    f.low_left = tank_flags[0].low;
    f.low_right = tank_flags[1].low;
    f.low_center = tank_flags[2].low;
    f.critical_left = tank_flags[0].critical;
    f.critical_right = tank_flags[1].critical;
    f.critical_center = tank_flags[2].critical;
    f.leak_suspected = tank_flags[0].leak || tank_flags[1].leak || tank_flags[2].leak;
    f.transfer_left = fuel_.transferred_left();
    f.transfer_right = fuel_.transferred_right();
    f.main_left = fuel_.level(0);
    f.main_right = fuel_.level(1);
    f.center = fuel_.level(2);
    f.total = fuel_.total();
    f.burn_rate = std::min(kMaxReportedKg, fuel_.burn_rate_kgh());
    f.transferred = std::min(kMaxReportedKg, fuel_.transferred_total());
    fuel_status_ = f;

    // The terminal's status.
    status_.autothrottle_engaged = thrust_.autothrottle;
    status_.asymmetric_thrust = thrust_.asymmetric();
    status_.low_fuel = f.low_left || f.low_right || f.low_center || f.critical_left || f.critical_right ||
                       f.critical_center;
    status_.critical_fuel = f.critical_left || f.critical_right || f.critical_center;
    status_.leak_suspected = f.leak_suspected;
    status_.engine_cmd_status = cmd_status_;
    status_.autothrottle_target = thrust_.autothrottle ? thrust_.target_kt : 0.0;
}

void EngineManagement::command_mode(const icd::EngModeCmd& cmd) noexcept {
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

void EngineManagement::command(const icd::EngCmd& cmd) noexcept {
    if (!cmd_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    cmd_status_ = icd::CmdStatus::REJECTED_INVALID;
    switch (cmd.command) {
        case icd::EngineCommand::SET_THRUST: {
            if (!std::isfinite(cmd.thrust) || cmd.thrust < 0.0 || cmd.thrust > 100.0) {
                break;
            }
            if (cmd.engine == icd::EngineSelect::ALL) {
                thrust_.set_all(cmd.thrust);
            } else if (cmd.engine == icd::EngineSelect::ENGINE_1) {
                thrust_.set_thrust(0, cmd.thrust);
            } else if (cmd.engine == icd::EngineSelect::ENGINE_2) {
                thrust_.set_thrust(1, cmd.thrust);
            } else {
                break;
            }
            commands_valid_ = true;
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            break;
        }
        case icd::EngineCommand::AUTOTHROTTLE_ENGAGE:
            if (std::isfinite(cmd.target_airspeed) && cmd.target_airspeed > 0.0) {
                thrust_.engage(cmd.target_airspeed);
                cmd_status_ = icd::CmdStatus::ACCEPTED;
            }
            break;
        case icd::EngineCommand::AUTOTHROTTLE_DISENGAGE:
            thrust_.disengage();
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            break;
        case icd::EngineCommand::FUEL_TRANSFER: {
            const std::size_t source = tank_index(cmd.source);
            const std::size_t destination = tank_index(cmd.destination);
            if (source < kTanks && destination < kTanks && source != destination && std::isfinite(cmd.amount) &&
                cmd.amount > 0.0) {
                const double moved = fuel_.transfer(source, destination, cmd.amount);
                cmd_status_ = moved > 0.0 ? icd::CmdStatus::ACCEPTED : icd::CmdStatus::FAILED;
            }
            break;
        }
        case icd::EngineCommand::NONE:
            break;
    }
    // The outputs follow at the next step; the status shows the outcome now.
    status_.engine_cmd_status = cmd_status_;
}

}  // namespace fmofp::engine
