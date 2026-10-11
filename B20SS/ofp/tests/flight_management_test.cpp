#include <gtest/gtest.h>

#include <cmath>

#include <fmofp/fms/flight_management.hpp>
#include <fmofp/icd/messages.hpp>

namespace {

using fmofp::Status;
using fmofp::fms::apply_autopilot_targets;
using fmofp::fms::apply_thrust;
using fmofp::fms::Attitude;
using fmofp::fms::AttitudeRequest;
using fmofp::fms::AutopilotTargets;
using fmofp::fms::energy_maneuverability;
using fmofp::fms::energy_state;
using fmofp::fms::EnergyMetrics;
using fmofp::fms::FlightManagement;
using fmofp::fms::limit_attitude;
using fmofp::fms::LimitedAttitude;
using fmofp::fms::mach_number;
using fmofp::fms::maneuver_request;
using fmofp::fms::profile;
using fmofp::fms::tactical_systems;
using fmofp::fms::ThrustState;
using fmofp::icd::CmdStatus;
using fmofp::icd::CountermeasuresState;
using fmofp::icd::FmsAttitudeCmd;
using fmofp::icd::FmsAttitudeIn;
using fmofp::icd::FmsControlCmd;
using fmofp::icd::FmsFlightMode;
using fmofp::icd::FmsFlightModeCmd;
using fmofp::icd::FmsManeuverCmd;
using fmofp::icd::FmsModeCmd;
using fmofp::icd::FmsNavSolution;
using fmofp::icd::FmsNavUpdateCmd;
using fmofp::icd::FmsSensors;
using fmofp::icd::GearPosition;
using fmofp::icd::Maneuver;
using fmofp::icd::Ssm;
using fmofp::icd::TargetingState;
using fmofp::icd::TerminalState;
using fmofp::icd::WeaponsState;

TEST(Profiles, AreTheFlightProfilesOfFmsControl) {
    EXPECT_DOUBLE_EQ(30.0, profile(FmsFlightMode::NORMAL).max_bank_angle);
    EXPECT_DOUBLE_EQ(3.0, profile(FmsFlightMode::NORMAL).turn_rate_limit);
    EXPECT_DOUBLE_EQ(80.0, profile(FmsFlightMode::COMBAT).max_bank_angle);
    EXPECT_DOUBLE_EQ(9.0, profile(FmsFlightMode::COMBAT).max_g_force);
    EXPECT_DOUBLE_EQ(0.6, profile(FmsFlightMode::STEALTH).engine_power);
    EXPECT_DOUBLE_EQ(22.0, profile(FmsFlightMode::TRAINING).max_aoa);
    EXPECT_DOUBLE_EQ(85.0, profile(FmsFlightMode::EMERGENCY).max_pitch_angle);
    EXPECT_DOUBLE_EQ(25.0, profile(FmsFlightMode::EMERGENCY).turn_rate_limit);

    const auto combat = tactical_systems(FmsFlightMode::COMBAT);
    EXPECT_EQ(CountermeasuresState::READY, combat.countermeasures);
    EXPECT_EQ(TargetingState::ACTIVE, combat.targeting);
    EXPECT_EQ(WeaponsState::ARMED, combat.weapons);
    EXPECT_FALSE(combat.stealth_mode);
    const auto stealth = tactical_systems(FmsFlightMode::STEALTH);
    EXPECT_EQ(CountermeasuresState::PASSIVE, stealth.countermeasures);
    EXPECT_TRUE(stealth.stealth_mode);
    const auto emergency = tactical_systems(FmsFlightMode::EMERGENCY);
    EXPECT_EQ(CountermeasuresState::ACTIVE, emergency.countermeasures);
    EXPECT_EQ(TargetingState::READY, emergency.targeting);
    EXPECT_EQ(WeaponsState::READY, emergency.weapons);
    const auto training = tactical_systems(FmsFlightMode::TRAINING);
    EXPECT_EQ(CountermeasuresState::STANDBY, training.countermeasures);
    EXPECT_EQ(WeaponsState::SAFE, training.weapons);
}

TEST(LimitAttitude, HoldsEachValueToTheProfileAndSaysWhich) {
    // BREAK_RIGHT in NORMAL: roll 60 -> 30, roll rate 40 -> 9, yaw rate 10 -> 3.
    AttitudeRequest r;
    r.roll_present = r.roll_rate_present = r.yaw_rate_present = true;
    r.value.roll = 60.0;
    r.value.roll_rate = 40.0;
    r.value.yaw_rate = 10.0;
    const Attitude current{2.0, 1.5, 270.0, 0.0, 0.0, 0.0};
    LimitedAttitude l = limit_attitude(r, current, profile(FmsFlightMode::NORMAL));
    EXPECT_DOUBLE_EQ(30.0, l.applied.roll);
    EXPECT_DOUBLE_EQ(9.0, l.applied.roll_rate);
    EXPECT_DOUBLE_EQ(3.0, l.applied.yaw_rate);
    EXPECT_DOUBLE_EQ(1.5, l.applied.pitch);  // absent: the current value
    EXPECT_DOUBLE_EQ(270.0, l.applied.yaw);
    EXPECT_DOUBLE_EQ(0.0, l.applied.pitch_rate);
    EXPECT_TRUE(l.bank_limited);
    EXPECT_TRUE(l.roll_rate_limited);
    EXPECT_TRUE(l.yaw_rate_limited);
    EXPECT_FALSE(l.pitch_limited);
    EXPECT_FALSE(l.pitch_rate_limited);

    // The same in COMBAT fits the envelope.
    l = limit_attitude(r, current, profile(FmsFlightMode::COMBAT));
    EXPECT_DOUBLE_EQ(60.0, l.applied.roll);
    EXPECT_DOUBLE_EQ(40.0, l.applied.roll_rate);
    EXPECT_DOUBLE_EQ(10.0, l.applied.yaw_rate);
    EXPECT_FALSE(l.bank_limited || l.roll_rate_limited || l.yaw_rate_limited);

    // Limits keep the sign; a current value over the limit is limited too.
    AttitudeRequest left;
    left.pitch_present = left.pitch_rate_present = true;
    left.value.pitch = -50.0;
    left.value.pitch_rate = -100.0;
    l = limit_attitude(left, Attitude{-40.0, 0.0, 0.0, 0.0, 0.0, 0.0}, profile(FmsFlightMode::NORMAL));
    EXPECT_DOUBLE_EQ(-20.0, l.applied.pitch);
    EXPECT_DOUBLE_EQ(-6.0, l.applied.pitch_rate);
    EXPECT_DOUBLE_EQ(-30.0, l.applied.roll);
    EXPECT_TRUE(l.pitch_limited && l.pitch_rate_limited && l.bank_limited);
}

TEST(Thrust, FollowsSetThrustIncludingTheAfterburnerThatNeverLights) {
    ThrustState t;
    EXPECT_DOUBLE_EQ(0.5, t.thrust);
    EXPECT_EQ(50, t.throttle_position);
    // Without the afterburner the most it allows is 0.9, and 0.9 is not above 0.9,
    // so set_thrust alone can never light it.
    EXPECT_DOUBLE_EQ(0.9, apply_thrust(t, 0.95));
    EXPECT_EQ(90, t.throttle_position);
    EXPECT_FALSE(t.afterburner);
    EXPECT_DOUBLE_EQ(0.1, apply_thrust(t, 0.0));
    EXPECT_EQ(10, t.throttle_position);
    EXPECT_DOUBLE_EQ(0.29, apply_thrust(t, 0.29));
    EXPECT_EQ(28, t.throttle_position);  // int(28.999999999999996), as in Python
    // Lit by other means, it stays lit down to 0.85 and allows 1.0.
    t.afterburner = true;
    EXPECT_DOUBLE_EQ(1.0, apply_thrust(t, 1.2));
    EXPECT_TRUE(t.afterburner);
    EXPECT_DOUBLE_EQ(0.87, apply_thrust(t, 0.87));
    EXPECT_TRUE(t.afterburner);
    EXPECT_DOUBLE_EQ(0.84, apply_thrust(t, 0.84));
    EXPECT_FALSE(t.afterburner);
}

TEST(Autopilot, LimitsThePresentTargetsAsSetAutopilotTargets) {
    AutopilotTargets t;
    FmsNavUpdateCmd cmd{};
    cmd.altitude_present = cmd.airspeed_present = cmd.heading_present = cmd.vertical_speed_present = true;
    cmd.altitude = 70000.0;
    cmd.airspeed = 50.0;
    cmd.heading = 725.0;
    cmd.vertical_speed = -9000.0;
    apply_autopilot_targets(t, cmd);
    EXPECT_DOUBLE_EQ(60000.0, t.altitude_ft);
    EXPECT_DOUBLE_EQ(100.0, t.airspeed_kt);
    EXPECT_DOUBLE_EQ(5.0, t.heading_deg);
    EXPECT_DOUBLE_EQ(-8000.0, t.vertical_speed_fpm);
    FmsNavUpdateCmd only_heading{};
    only_heading.heading_present = true;
    only_heading.heading = -10.0;
    apply_autopilot_targets(t, only_heading);
    EXPECT_DOUBLE_EQ(350.0, t.heading_deg);
    EXPECT_DOUBLE_EQ(60000.0, t.altitude_ft);  // absent: unchanged
}

TEST(Maneuvers, AreTheRequestsExecuteManeuverMakes) {
    AttitudeRequest r;
    EXPECT_EQ(CmdStatus::COMPLETED, maneuver_request(Maneuver::BREAK_LEFT, 0.0, profile(FmsFlightMode::NORMAL), r));
    EXPECT_TRUE(r.roll_present && r.roll_rate_present && r.yaw_rate_present && !r.pitch_present);
    EXPECT_DOUBLE_EQ(-60.0, r.value.roll);
    EXPECT_DOUBLE_EQ(-40.0, r.value.roll_rate);
    EXPECT_DOUBLE_EQ(-10.0, r.value.yaw_rate);
    EXPECT_EQ(CmdStatus::COMPLETED, maneuver_request(Maneuver::MAXIMUM_CLIMB, 0.0, profile(FmsFlightMode::TRAINING), r));
    EXPECT_DOUBLE_EQ(45.0, r.value.pitch);
    EXPECT_EQ(CmdStatus::COMPLETED, maneuver_request(Maneuver::DIVE, 0.0, profile(FmsFlightMode::NORMAL), r));
    EXPECT_DOUBLE_EQ(-30.0, r.value.pitch);  // the default angle
    EXPECT_EQ(CmdStatus::COMPLETED, maneuver_request(Maneuver::DIVE, -12.5, profile(FmsFlightMode::NORMAL), r));
    EXPECT_DOUBLE_EQ(-12.5, r.value.pitch);
    EXPECT_EQ(CmdStatus::COMPLETED,
              maneuver_request(Maneuver::DEFENSIVE_SPLIT_S, 0.0, profile(FmsFlightMode::NORMAL), r));
    EXPECT_DOUBLE_EQ(180.0, r.value.roll);
    EXPECT_DOUBLE_EQ(-80.0, r.value.pitch);
    EXPECT_EQ(CmdStatus::NOT_IMPLEMENTED, maneuver_request(Maneuver::BARREL_ROLL, 0.0, profile(FmsFlightMode::NORMAL), r));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, maneuver_request(Maneuver::NONE, 0.0, profile(FmsFlightMode::NORMAL), r));
}

TEST(Derived, MachEnergyStateAndEnergyManeuverability) {
    EXPECT_DOUBLE_EQ(450.0 / (661.4788 * 0.98), mach_number(450.0));
    // 450 kt at 30 000 ft: kinetic 0.5 x 15000 x 231.498^2, potential 15000 x 9.81 x 9144.
    const double v = 450.0 * 0.51444;
    const double expected = (0.5 * 15000.0 * v * v + 15000.0 * 9.81 * 30000.0 * 0.3048) / 1e10 * 100.0;
    EXPECT_DOUBLE_EQ(expected, energy_state(450.0, 30000.0));
    EXPECT_LT(energy_state(1000.0, 60000.0), 100.0);  // the envelope never reaches the cap
    EXPECT_DOUBLE_EQ(100.0, energy_state(5000.0, 60000.0));  // but the cap is there

    const EnergyMetrics m = energy_maneuverability(30000.0, 450.0, 2.0, profile(FmsFlightMode::COMBAT));
    const double rho = 1.225 * std::exp(-30000.0 * 0.3048 / 10000.0);
    const double drag = 0.5 * rho * v * v * (0.02 + 0.02) * 30.0;
    const double weight = 15000.0 * 9.81;
    EXPECT_NEAR(v * (150000.0 - drag) / weight, m.specific_excess_power, 1e-9);
    const double twr = 150000.0 / weight;
    EXPECT_NEAR(9.81 * std::sqrt(twr * twr - 1.0) / v * 180.0 / 3.141592653589793, m.max_sustained_turn_rate, 1e-9);
    EXPECT_NEAR(9.81 * std::sqrt(81.0 - 1.0) / v * 180.0 / 3.141592653589793, m.max_instantaneous_turn_rate, 1e-9);
    EXPECT_NEAR(m.specific_excess_power * 196.85, m.max_climb_rate_fpm, 1e-9);
    EXPECT_NEAR(m.specific_excess_power * 10.0, m.energy_advantage, 1e-9);
    // NORMAL's 0.7 engine power gives a thrust/weight ratio below 1: no sustained turn.
    EXPECT_DOUBLE_EQ(0.0, energy_maneuverability(30000.0, 450.0, 2.0, profile(FmsFlightMode::NORMAL)).max_sustained_turn_rate);
}

FmsSensors sensors() {
    FmsSensors s{};
    for (Ssm* ssm : {&s.payload.altitude_ssm, &s.payload.airspeed_ssm, &s.payload.pitch_ssm, &s.payload.roll_ssm,
                     &s.payload.heading_ssm, &s.payload.vertical_speed_ssm, &s.payload.aoa_ssm, &s.payload.g_ssm}) {
        *ssm = Ssm::NORMAL;
    }
    s.payload.altitude = 30000.0;
    s.payload.airspeed = 450.0;
    s.payload.heading = 90.0;
    s.payload.vertical_speed = -300.0;
    s.payload.angle_of_attack = 2.0;
    s.payload.g_force = 1.0;
    return s;
}

FmsAttitudeIn attitude(double roll, double pitch, double yaw) {
    FmsAttitudeIn a{};
    a.payload.roll_ssm = a.payload.pitch_ssm = a.payload.yaw_ssm = Ssm::NORMAL;
    a.payload.roll = roll;
    a.payload.pitch = pitch;
    a.payload.yaw = yaw;
    return a;
}

FmsNavSolution nav(double lat, double lon, double alt) {
    FmsNavSolution n{};
    n.payload.position_ssm = n.payload.altitude_ssm = Ssm::NORMAL;
    n.payload.gps_valid = true;
    n.payload.latitude = lat;
    n.payload.longitude = lon;
    n.payload.altitude = alt;
    return n;
}

TEST(FlightManagement, FlightDataFollowsItsInputsAndTheirValidity) {
    FlightManagement fm;
    ASSERT_EQ(Status::kOk, fm.init());
    EXPECT_EQ(TerminalState::INITIALIZING, fm.state());
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fm.flight_data().airspeed_ssm);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fm.attitude_status().roll_ssm);
    EXPECT_EQ(CmdStatus::NONE, fm.attitude_status().status);
    EXPECT_DOUBLE_EQ(30000.0, fm.nav_update_status().altitude);  // the Python's initial targets
    EXPECT_DOUBLE_EQ(450.0, fm.nav_update_status().airspeed);
    EXPECT_DOUBLE_EQ(0.5, fm.control_status().thrust);
    EXPECT_EQ(GearPosition::UP, fm.control_status().gear);

    const FmsSensors s = sensors();
    const FmsAttitudeIn a = attitude(-10.0, 2.5, -90.0);
    const FmsNavSolution n = nav(0.01, 0.02, 29950.0);
    fm.step(&s, &a, &n, 1);
    EXPECT_EQ(TerminalState::NORMAL, fm.state());
    const auto& d = fm.flight_data();
    EXPECT_EQ(Ssm::NORMAL, d.roll_ssm);
    EXPECT_DOUBLE_EQ(-10.0, d.roll);
    EXPECT_DOUBLE_EQ(2.5, d.pitch);
    EXPECT_DOUBLE_EQ(90.0, d.heading);
    EXPECT_DOUBLE_EQ(450.0, d.airspeed);
    EXPECT_DOUBLE_EQ(450.0, d.ground_speed);
    EXPECT_DOUBLE_EQ(mach_number(450.0), d.mach);
    EXPECT_DOUBLE_EQ(30000.0, d.altitude);
    EXPECT_DOUBLE_EQ(29950.0, d.nav_altitude);
    EXPECT_DOUBLE_EQ(0.01, d.latitude);
    EXPECT_DOUBLE_EQ(0.02, d.longitude);
    EXPECT_EQ(Ssm::NORMAL, d.position_ssm);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, d.track_ssm);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, d.sideslip_ssm);
    EXPECT_EQ(Ssm::NORMAL, d.energy_ssm);
    EXPECT_DOUBLE_EQ(energy_state(450.0, 29950.0), d.energy_state);
    EXPECT_EQ(Ssm::NORMAL, fm.energy().ssm);
    EXPECT_EQ(FmsFlightMode::NORMAL, d.flight_mode);

    // The attitude stops: its fields go NCD after three frames, the rest stays.
    fm.step(&s, nullptr, nullptr, 2);
    fm.step(&s, nullptr, nullptr, 3);
    EXPECT_EQ(Ssm::NORMAL, fm.flight_data().roll_ssm);
    fm.step(&s, nullptr, nullptr, 4);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fm.flight_data().roll_ssm);
    EXPECT_EQ(Ssm::NORMAL, fm.flight_data().airspeed_ssm);
    EXPECT_EQ(Ssm::NORMAL, fm.flight_data().position_ssm);  // 10 Hz: fresh for 15 frames
    for (std::uint32_t frame = 5; frame <= 16; ++frame) {
        fm.step(&s, nullptr, nullptr, frame);
    }
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fm.flight_data().position_ssm);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fm.flight_data().energy_ssm);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fm.energy().ssm);

    // The sensors stop: DEGRADED, and NORMAL again when they return.
    fm.step(nullptr, nullptr, nullptr, 17);
    fm.step(nullptr, nullptr, nullptr, 18);
    fm.step(nullptr, nullptr, nullptr, 19);
    EXPECT_EQ(TerminalState::DEGRADED, fm.state());
    fm.step(&s, nullptr, nullptr, 20);
    EXPECT_EQ(TerminalState::NORMAL, fm.state());
}

TEST(FlightManagement, CommandsFollowTheSequenceRuleAndTheModeInForce) {
    FlightManagement fm;
    ASSERT_EQ(Status::kOk, fm.init());
    const FmsSensors s = sensors();
    const FmsAttitudeIn a = attitude(5.0, 1.0, 0.0);
    fm.step(&s, &a, nullptr, 1);

    // A maneuver in NORMAL is limited; absent fields keep the measured attitude.
    FmsManeuverCmd m{};
    m.cmd_seq = 1;
    m.maneuver = Maneuver::BREAK_RIGHT;
    fm.command_maneuver(m);
    EXPECT_EQ(CmdStatus::COMPLETED, fm.maneuver_status().status);
    EXPECT_EQ(Maneuver::BREAK_RIGHT, fm.maneuver_status().maneuver);
    EXPECT_EQ(1, fm.maneuver_status().last_cmd_seq);
    EXPECT_DOUBLE_EQ(30.0, fm.attitude_status().roll);
    EXPECT_DOUBLE_EQ(1.0, fm.attitude_status().pitch);
    EXPECT_DOUBLE_EQ(9.0, fm.attitude_status().roll_rate);
    EXPECT_DOUBLE_EQ(3.0, fm.attitude_status().yaw_rate);
    EXPECT_TRUE(fm.attitude_status().bank_limited);
    EXPECT_EQ(Ssm::NORMAL, fm.attitude_status().roll_ssm);
    EXPECT_EQ(0, fm.attitude_status().last_cmd_seq);  // that is FMS_ATTITUDE_CMD's

    // The same seq again is ignored; BARREL_ROLL is not implemented and changes nothing.
    m.maneuver = Maneuver::BARREL_ROLL;
    fm.command_maneuver(m);
    EXPECT_EQ(Maneuver::BREAK_RIGHT, fm.maneuver_status().maneuver);
    m.cmd_seq = 2;
    fm.command_maneuver(m);
    EXPECT_EQ(CmdStatus::NOT_IMPLEMENTED, fm.maneuver_status().status);
    EXPECT_DOUBLE_EQ(30.0, fm.attitude_status().roll);

    // COMBAT: the flight mode command changes the limits and the tactical systems.
    FmsFlightModeCmd mode{};
    mode.cmd_seq = 1;
    mode.flight_mode = FmsFlightMode::COMBAT;
    fm.command_flight_mode(mode);
    fm.step(&s, &a, nullptr, 2);
    EXPECT_EQ(FmsFlightMode::COMBAT, fm.status().flight_mode);
    EXPECT_EQ(CmdStatus::COMPLETED, fm.status().flight_mode_status);
    EXPECT_EQ(1, fm.status().flight_mode_cmd_seq);
    EXPECT_EQ(WeaponsState::ARMED, fm.status().weapons);
    EXPECT_EQ(FmsFlightMode::COMBAT, fm.flight_data().flight_mode);
    FmsAttitudeCmd att{};
    att.cmd_seq = 1;
    att.roll_present = att.pitch_present = att.roll_rate_present = true;
    att.roll = 45.0;
    att.pitch = 10.0;
    att.roll_rate = 50.0;
    fm.command_attitude(att);
    EXPECT_EQ(CmdStatus::COMPLETED, fm.attitude_status().status);
    EXPECT_EQ(1, fm.attitude_status().last_cmd_seq);
    EXPECT_DOUBLE_EQ(45.0, fm.attitude_status().roll);
    EXPECT_DOUBLE_EQ(10.0, fm.attitude_status().pitch);
    EXPECT_DOUBLE_EQ(50.0, fm.attitude_status().roll_rate);
    EXPECT_DOUBLE_EQ(0.0, fm.attitude_status().yaw_rate);  // rates not sent are 0, the idle value
    EXPECT_FALSE(fm.attitude_status().bank_limited || fm.attitude_status().roll_rate_limited);

    // Autopilot targets and thrust/gear.
    FmsNavUpdateCmd nav_cmd{};
    nav_cmd.cmd_seq = 1;
    nav_cmd.altitude_present = nav_cmd.airspeed_present = true;
    nav_cmd.altitude = 70000.0;
    nav_cmd.airspeed = 50.0;
    fm.command_nav_update(nav_cmd);
    fm.step(&s, &a, nullptr, 3);
    EXPECT_EQ(CmdStatus::COMPLETED, fm.nav_update_status().status);
    EXPECT_DOUBLE_EQ(60000.0, fm.nav_update_status().altitude);
    EXPECT_DOUBLE_EQ(100.0, fm.nav_update_status().airspeed);
    EXPECT_DOUBLE_EQ(0.0, fm.nav_update_status().heading);
    FmsControlCmd ctl{};
    ctl.cmd_seq = 1;
    ctl.thrust_present = ctl.gear_present = true;
    ctl.thrust = 0.95;
    ctl.gear = GearPosition::DOWN;
    fm.command_control(ctl);
    fm.step(&s, &a, nullptr, 4);
    EXPECT_EQ(CmdStatus::COMPLETED, fm.control_status().status);
    EXPECT_DOUBLE_EQ(0.9, fm.control_status().thrust);
    EXPECT_EQ(90, fm.control_status().throttle_position);
    EXPECT_FALSE(fm.control_status().afterburner);
    EXPECT_EQ(GearPosition::DOWN, fm.control_status().gear);
    ctl.cmd_seq = 2;
    ctl.gear = GearPosition::IN_TRANSIT;
    fm.command_control(ctl);
    fm.step(&s, &a, nullptr, 5);
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, fm.control_status().status);
    EXPECT_EQ(GearPosition::DOWN, fm.control_status().gear);  // nothing applied
    EXPECT_DOUBLE_EQ(0.9, fm.control_status().thrust);

    // Terminal state commands.
    FmsModeCmd tcmd{};
    tcmd.cmd_seq = 1;
    tcmd.requested_state = TerminalState::FAILURE;
    fm.command_mode(tcmd);
    EXPECT_TRUE(fm.status().cmd_rejected);
    EXPECT_EQ(TerminalState::NORMAL, fm.state());
    tcmd.cmd_seq = 2;
    tcmd.requested_state = TerminalState::MAINTENANCE;
    fm.command_mode(tcmd);
    EXPECT_FALSE(fm.status().cmd_rejected);
    EXPECT_EQ(TerminalState::MAINTENANCE, fm.state());
}

}  // namespace
