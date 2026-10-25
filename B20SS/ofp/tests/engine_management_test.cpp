// The engine management logic against the Python it replaces: thrustManagement.py's
// per-engine thrust, autothrottle and asymmetry check, fuelControl.py's tanks, fuel
// lines and transfer policy with fuelMonitor.py's flags, and the partition's state,
// staleness and command handling the ICD defines.
#include <gtest/gtest.h>

#include <cstdint>

#include <fmofp/engine/engine_management.hpp>

namespace {

using fmofp::Status;
using fmofp::engine::EngineManagement;
using fmofp::engine::FuelSystem;
using fmofp::engine::ThrustManager;
using fmofp::engine::kAutothrottleFrames;
using fmofp::engine::kEngineStaleFrames;
using namespace fmofp::icd;  // NOLINT: test

// engineControlUnit._adjust without its noise, for both engines at one thrust.
PlantEngine engines_at(double thrust) {
    PlantEngine p{};
    p.engine_1_ssm = Ssm::NORMAL;
    p.engine_2_ssm = Ssm::NORMAL;
    p.thrust_1 = p.thrust_2 = thrust;
    p.n1_1 = p.n1_2 = 0.80 * thrust + 18.0;
    p.n2_1 = p.n2_2 = 0.75 * thrust + 22.0;
    p.egt_1 = p.egt_2 = 350.0 + thrust * 4.5;
    p.fuel_flow_1 = p.fuel_flow_2 = thrust * 42.0;
    p.oil_pressure_1 = p.oil_pressure_2 = 60.0;
    p.oil_temperature_1 = p.oil_temperature_2 = 92.0;
    p.vibration_1 = p.vibration_2 = 0.3;
    p.compressor_efficiency_1 = p.compressor_efficiency_2 = 88.0;
    return p;
}

EngEngineIn as_in(const PlantEngine& p) {
    EngEngineIn in{};
    in.payload = p;
    return in;
}

// FMS_CONTROL_STATUS as relayed: the thrust lever at a percent.
EngFmsControl lever(std::uint8_t throttle, bool afterburner = false) {
    EngFmsControl c{};
    c.payload.throttle_position = throttle;
    c.payload.afterburner = afterburner;
    c.payload.gear = GearPosition::UP;
    c.payload.status = CmdStatus::COMPLETED;
    return c;
}

BcastFmsFlightData airspeed(double kt) {
    BcastFmsFlightData f{};
    f.payload.airspeed_ssm = Ssm::NORMAL;
    f.payload.airspeed = kt;
    return f;
}

EngCmd set_thrust(std::uint8_t seq, EngineSelect engine, double pct) {
    EngCmd c{};
    c.cmd_seq = seq;
    c.command = EngineCommand::SET_THRUST;
    c.engine = engine;
    c.thrust = pct;
    return c;
}

EngCmd autothrottle(std::uint8_t seq, double target) {
    EngCmd c{};
    c.cmd_seq = seq;
    c.command = EngineCommand::AUTOTHROTTLE_ENGAGE;
    c.target_airspeed = target;
    return c;
}

EngCmd simple(std::uint8_t seq, EngineCommand command) {
    EngCmd c{};
    c.cmd_seq = seq;
    c.command = command;
    return c;
}

EngCmd transfer(std::uint8_t seq, FuelTank from, FuelTank to, double kg) {
    EngCmd c{};
    c.cmd_seq = seq;
    c.command = EngineCommand::FUEL_TRANSFER;
    c.source = from;
    c.destination = to;
    c.amount = kg;
    return c;
}

TEST(ThrustManagerTest, SetThrustClampsAndTheAsymmetryIsMaxMinusMin) {
    ThrustManager t;
    EXPECT_DOUBLE_EQ(0.0, t.thrust[0]) << "register_engine started both at 0";
    t.set_thrust(0, 150.0);
    t.set_thrust(1, -5.0);
    EXPECT_DOUBLE_EQ(100.0, t.thrust[0]);
    EXPECT_DOUBLE_EQ(0.0, t.thrust[1]);
    EXPECT_DOUBLE_EQ(100.0, t.asymmetry());
    EXPECT_TRUE(t.asymmetric());
    t.set_thrust(5, 50.0);  // no such engine: ignored, as set_thrust returned False
    EXPECT_DOUBLE_EQ(100.0, t.thrust[0]);
    t.set_all(60.0);
    EXPECT_DOUBLE_EQ(60.0, t.thrust[0]);
    EXPECT_DOUBLE_EQ(60.0, t.thrust[1]);
    t.set_thrust(1, 35.0);
    EXPECT_DOUBLE_EQ(25.0, t.asymmetry());
    EXPECT_FALSE(t.asymmetric()) << "the limit itself is not above it";
    t.set_thrust(1, 34.9);
    EXPECT_TRUE(t.asymmetric());
}

TEST(ThrustManagerTest, TheAutothrottleIsThePythonsPiLoop) {
    ThrustManager t;
    t.set_all(60.0);
    t.update_autothrottle(450.0, 0.5);
    EXPECT_DOUBLE_EQ(60.0, t.thrust[0]) << "a no-op unless engaged";
    t.engage(500.0);
    EXPECT_TRUE(t.autothrottle);
    t.update_autothrottle(450.0, 0.0);
    EXPECT_DOUBLE_EQ(60.0, t.thrust[0]) << "a no-op for dt 0";
    // error 50: integral 25, delta 0.5 x 50 + 0.05 x 25 = 26.25 on the 60 average.
    t.update_autothrottle(450.0, 0.5);
    EXPECT_DOUBLE_EQ(86.25, t.thrust[0]);
    EXPECT_DOUBLE_EQ(86.25, t.thrust[1]);
    EXPECT_DOUBLE_EQ(25.0, t.integral);
    EXPECT_DOUBLE_EQ(50.0, t.last_error);
    // integral capped at 50: delta 27.5, the average 86.25 + 27.5 is clamped to 100.
    t.update_autothrottle(450.0, 0.5);
    EXPECT_DOUBLE_EQ(100.0, t.thrust[0]);
    EXPECT_DOUBLE_EQ(50.0, t.integral);
    t.update_autothrottle(450.0, 0.5);
    EXPECT_DOUBLE_EQ(50.0, t.integral) << "the cap holds";
    // error -20: integral 40, delta -10 + 2 = -8.
    t.update_autothrottle(520.0, 0.5);
    EXPECT_DOUBLE_EQ(92.0, t.thrust[0]);
    EXPECT_DOUBLE_EQ(40.0, t.integral);
    EXPECT_DOUBLE_EQ(-20.0, t.last_error);
    t.disengage();
    EXPECT_FALSE(t.autothrottle);
    EXPECT_DOUBLE_EQ(0.0, t.target_kt);
    t.update_autothrottle(100.0, 0.5);
    EXPECT_DOUBLE_EQ(92.0, t.thrust[0]) << "the thrust stays where the autothrottle left it";
    t.engage(300.0);
    EXPECT_DOUBLE_EQ(0.0, t.integral) << "engaging resets the integrator";
}

TEST(FuelSystemTest, TheBurnFollowsTheFuelLines) {
    FuelSystem fs;
    fs.init();
    EXPECT_DOUBLE_EQ(40000.0, fs.total()) << "Main Left 10000, Main Right 10000, Center 20000, full";
    // Engine 1 at 3600 kg/h for one second: 1 kg, split Main Left 100 : Center 50.
    fs.step({3600.0, 0.0}, 1.0);
    EXPECT_NEAR(10000.0 - 2.0 / 3.0, fs.level(0), 1e-9);
    EXPECT_DOUBLE_EQ(10000.0, fs.level(1));
    EXPECT_NEAR(20000.0 - 1.0 / 3.0, fs.level(2), 1e-9);
    EXPECT_DOUBLE_EQ(0.0, fs.burn_rate_kgh()) << "the first poll has nothing to compare with";
    fs.step({3600.0, 3600.0}, 1.0);
    EXPECT_NEAR(7200.0, fs.burn_rate_kgh(), 1e-6) << "the monitor's rates: the drop over the interval";
    EXPECT_NEAR(10000.0 - 2.0 / 3.0, fs.level(1), 1e-9) << "engine 2 draws from Main Right and Center";
    EXPECT_FALSE(fs.flags()[0].leak || fs.flags()[1].leak || fs.flags()[2].leak);
    const double before = fs.total();
    fs.step({-10.0, 0.0}, 1.0);
    EXPECT_DOUBLE_EQ(before, fs.total()) << "a negative flow burns nothing";
    fs.step({3600.0, 0.0}, 0.0);
    EXPECT_DOUBLE_EQ(before, fs.total()) << "nor does a pass of no duration";
    fs.set_level(2, 5000.0);  // not above 5000: the transfer policy leaves the drained main alone
    fs.set_level(0, 0.0);
    fs.step({3600.0, 0.0}, 1.0);
    EXPECT_DOUBLE_EQ(0.0, fs.level(0)) << "an empty tank cannot go negative";
    EXPECT_NEAR(5000.0 - 1.0 / 3.0, fs.level(2), 1e-9) << "the Center still feeds its share";
}

TEST(FuelSystemTest, ATransferMovesWhatTheSourceHoldsAndTheDestinationCanTake) {
    FuelSystem fs;
    fs.init();
    EXPECT_DOUBLE_EQ(0.0, fs.transfer(2, 0, 1000.0)) << "Main Left is full";
    fs.set_level(0, 9500.0);
    EXPECT_DOUBLE_EQ(500.0, fs.transfer(2, 0, 1000.0));
    EXPECT_DOUBLE_EQ(10000.0, fs.level(0));
    EXPECT_DOUBLE_EQ(19500.0, fs.level(2));
    EXPECT_DOUBLE_EQ(500.0, fs.transferred_total());
    EXPECT_DOUBLE_EQ(0.0, fs.transfer(0, 0, 10.0)) << "same tank";
    EXPECT_DOUBLE_EQ(0.0, fs.transfer(0, 1, -5.0)) << "nothing to move";
    EXPECT_DOUBLE_EQ(0.0, fs.transfer(3, 1, 5.0)) << "no such tank";
    fs.set_level(2, 300.0);
    fs.set_level(1, 9000.0);
    EXPECT_DOUBLE_EQ(300.0, fs.transfer(2, 1, 1000.0)) << "the source runs dry";
    EXPECT_DOUBLE_EQ(0.0, fs.level(2));
    EXPECT_DOUBLE_EQ(9300.0, fs.level(1));
}

TEST(FuelSystemTest, ThePolicyTopsUpAMainBelow5000FromACenterAbove5000) {
    FuelSystem fs;
    fs.init();
    fs.set_level(0, 4000.0);
    fs.set_level(1, 4500.0);
    fs.set_level(2, 6000.0);
    fs.step({0.0, 0.0}, 0.1);
    EXPECT_DOUBLE_EQ(5000.0, fs.level(0));
    EXPECT_DOUBLE_EQ(5500.0, fs.level(1));
    EXPECT_DOUBLE_EQ(4000.0, fs.level(2)) << "both transfers decided on the levels read before either";
    EXPECT_TRUE(fs.transferred_left());
    EXPECT_TRUE(fs.transferred_right());
    EXPECT_DOUBLE_EQ(2000.0, fs.transferred_total());
    fs.step({0.0, 0.0}, 0.1);
    EXPECT_FALSE(fs.transferred_left()) << "the Center is at 5000: not above it";
    EXPECT_DOUBLE_EQ(5000.0, fs.level(0));
    EXPECT_FALSE(fs.flags()[0].leak || fs.flags()[2].leak);
    EXPECT_DOUBLE_EQ(0.0, fs.burn_rate_kgh()) << "transfers are not consumption";
    fs.set_level(2, 20000.0);
    fs.set_level(0, 4999.5);
    fs.step({0.0, 0.0}, 0.1);
    EXPECT_DOUBLE_EQ(5999.5, fs.level(0));
    EXPECT_TRUE(fs.transferred_left());
    EXPECT_FALSE(fs.transferred_right()) << "Main Right is at 5500";
}

TEST(FuelSystemTest, TheMonitorFlagsLowCriticalAndUnexplainedLoss) {
    FuelSystem fs;
    fs.init();
    fs.set_level(2, 5000.0);  // 25 %: neither low nor, at exactly 5000, a source for the transfer policy
    fs.set_level(0, 1500.0);
    fs.step({0.0, 0.0}, 0.1);
    EXPECT_TRUE(fs.flags()[0].low) << "15 % is at or below 20";
    EXPECT_FALSE(fs.flags()[0].critical);
    fs.set_level(0, 1000.0);
    fs.step({0.0, 0.0}, 0.1);
    EXPECT_TRUE(fs.flags()[0].critical) << "10 % is at or below 10";
    EXPECT_FALSE(fs.flags()[0].low) << "critical, not low, as the Python's elif had it";
    fs.set_level(0, 2001.0);
    fs.step({0.0, 0.0}, 0.1);
    EXPECT_FALSE(fs.flags()[0].low || fs.flags()[0].critical);
    EXPECT_FALSE(fs.flags()[1].low || fs.flags()[2].low) << "a full main and a quarter-full Center";

    FuelSystem leak;
    leak.init();
    leak.step({0.0, 0.0}, 0.1);
    leak.set_level(0, 9850.0);  // 150 kg gone: within 2 % of 10000
    leak.step({0.0, 0.0}, 0.1);
    EXPECT_FALSE(leak.flags()[0].leak);
    leak.set_level(0, 9600.0);  // 250 kg gone, nothing explains it
    leak.step({0.0, 0.0}, 0.1);
    EXPECT_TRUE(leak.flags()[0].leak);
    leak.step({0.0, 0.0}, 0.1);
    EXPECT_TRUE(leak.flags()[0].leak) << "latched";
    EXPECT_DOUBLE_EQ(0.0, leak.burn_rate_kgh()) << "the rate is the last pass's drop, and there was none";

    FuelSystem explained;
    explained.init();
    explained.set_level(2, 19000.0);
    explained.step({0.0, 0.0}, 0.1);
    EXPECT_DOUBLE_EQ(1000.0, explained.transfer(0, 2, 1000.0));
    explained.step({0.0, 0.0}, 0.1);
    EXPECT_FALSE(explained.flags()[0].leak) << "a transfer out explains the drop";
    EXPECT_DOUBLE_EQ(0.0, explained.burn_rate_kgh());
    explained.set_level(0, 8700.0);  // a further 300 kg, unexplained
    explained.step({0.0, 0.0}, 0.1);
    EXPECT_TRUE(explained.flags()[0].leak);
}

TEST(EngineManagementTest, TheStateFollowsTheEngineData) {
    EngineManagement em;
    ASSERT_EQ(Status::kOk, em.init());
    EXPECT_EQ(TerminalState::INITIALIZING, em.state());
    EXPECT_FALSE(em.thrust_cmd().commands_valid);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, em.engine_status().engine_1_ssm);
    EXPECT_DOUBLE_EQ(40000.0, em.fuel_status().total);
    EXPECT_EQ(CmdStatus::NONE, em.status().engine_cmd_status);

    const EngEngineIn in = as_in(engines_at(80.0));
    em.step(nullptr, nullptr, &in, 1);
    EXPECT_EQ(TerminalState::NORMAL, em.state());
    EXPECT_EQ(Ssm::NORMAL, em.engine_status().engine_1_ssm);
    EXPECT_DOUBLE_EQ(82.0, em.engine_status().n1_1);
    EXPECT_DOUBLE_EQ(6720.0, em.engine_status().total_fuel_flow);
    EXPECT_FALSE(em.thrust_cmd().commands_valid) << "nothing has commanded the thrust";
    for (std::uint32_t frame = 2; frame < 1 + kEngineStaleFrames; ++frame) {
        em.step(nullptr, nullptr, nullptr, frame);
    }
    EXPECT_EQ(TerminalState::NORMAL, em.state()) << "within three periods";
    em.step(nullptr, nullptr, nullptr, 1 + kEngineStaleFrames);
    EXPECT_EQ(TerminalState::DEGRADED, em.state());
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, em.engine_status().engine_1_ssm);
    EXPECT_DOUBLE_EQ(0.0, em.engine_status().n1_1);
    EXPECT_DOUBLE_EQ(0.0, em.engine_status().total_fuel_flow);
    em.step(nullptr, nullptr, &in, 1 + kEngineStaleFrames + 1);
    EXPECT_EQ(TerminalState::NORMAL, em.state());

    EngModeCmd mode{};
    mode.cmd_seq = 1;
    mode.requested_state = TerminalState::STANDBY;
    em.command_mode(mode);
    EXPECT_EQ(TerminalState::STANDBY, em.state());
    EXPECT_FALSE(em.status().cmd_rejected);
    EXPECT_EQ(1, em.status().last_cmd_seq);
    mode.cmd_seq = 2;
    mode.requested_state = TerminalState::FAILURE;
    em.command_mode(mode);
    EXPECT_TRUE(em.status().cmd_rejected);
    EXPECT_EQ(TerminalState::STANDBY, em.state());
    mode.requested_state = TerminalState::NORMAL;
    em.command_mode(mode);
    EXPECT_EQ(TerminalState::STANDBY, em.state()) << "the same cmd_seq again is not a new command";
    em.reset();
    EXPECT_EQ(TerminalState::INITIALIZING, em.state());
    EXPECT_FALSE(em.status().cmd_rejected);
}

TEST(EngineManagementTest, TheFmsLeverCommandsTheEnginesUntilTheAutothrottleOwnsThem) {
    EngineManagement em;
    ASSERT_EQ(Status::kOk, em.init());
    const EngEngineIn in = as_in(engines_at(50.0));
    EngFmsControl l = lever(50);
    em.step(&l, nullptr, &in, 1);
    EXPECT_TRUE(em.thrust_cmd().commands_valid);
    EXPECT_DOUBLE_EQ(50.0, em.thrust_cmd().thrust_1);
    EXPECT_DOUBLE_EQ(50.0, em.thrust_cmd().thrust_2);
    EXPECT_FALSE(em.thrust_cmd().afterburner);
    em.command(set_thrust(1, EngineSelect::ENGINE_2, 30.0));
    em.step(&l, nullptr, &in, 2);
    EXPECT_DOUBLE_EQ(30.0, em.thrust_cmd().thrust_2) << "the lever has not moved: the engine command stands";
    l = lever(80, true);
    em.step(&l, nullptr, &in, 3);
    EXPECT_DOUBLE_EQ(80.0, em.thrust_cmd().thrust_1);
    EXPECT_DOUBLE_EQ(80.0, em.thrust_cmd().thrust_2) << "a new lever position commands both engines";
    EXPECT_TRUE(em.thrust_cmd().afterburner);

    em.command(autothrottle(2, 500.0));
    EXPECT_EQ(CmdStatus::ACCEPTED, em.status().engine_cmd_status);
    l = lever(30);
    em.step(&l, nullptr, &in, 4);
    EXPECT_DOUBLE_EQ(80.0, em.thrust_cmd().thrust_1) << "the autothrottle owns the levers";
    EXPECT_TRUE(em.thrust_cmd().autothrottle_engaged);
    EXPECT_DOUBLE_EQ(500.0, em.status().autothrottle_target);
    const BcastFmsFlightData slow = airspeed(450.0);
    em.step(nullptr, &slow, &in, kAutothrottleFrames - 1);
    EXPECT_DOUBLE_EQ(80.0, em.thrust_cmd().thrust_1) << "the PI loop runs at 2 Hz";
    em.step(nullptr, nullptr, &in, kAutothrottleFrames);
    EXPECT_DOUBLE_EQ(100.0, em.thrust_cmd().thrust_1) << "80 + 26.25, clamped";
    EXPECT_DOUBLE_EQ(100.0, em.thrust_cmd().thrust_2);
    EXPECT_FALSE(em.thrust_cmd().asymmetric_thrust);
    // Without a fresh airspeed the loop does not run.
    em.command(set_thrust(3, EngineSelect::ALL, 60.0));
    for (std::uint32_t frame = kAutothrottleFrames + 1; frame <= 2 * kAutothrottleFrames; ++frame) {
        em.step(nullptr, nullptr, &in, frame);
    }
    EXPECT_DOUBLE_EQ(60.0, em.thrust_cmd().thrust_1) << "the airspeed went stale";
    const BcastFmsFlightData fast = airspeed(560.0);
    em.step(nullptr, &fast, &in, 3 * kAutothrottleFrames - 1);
    em.step(nullptr, nullptr, &in, 3 * kAutothrottleFrames);
    // error -60 on the integral of 25 left from the first step: 25 - 30 = -5; delta -30 - 0.25 on the 60 average.
    EXPECT_NEAR(60.0 - 30.25, em.thrust_cmd().thrust_1, 1e-9);
    EXPECT_NEAR(-5.0, em.thrust().integral, 1e-9);

    em.command(simple(4, EngineCommand::AUTOTHROTTLE_DISENGAGE));
    EXPECT_EQ(CmdStatus::ACCEPTED, em.status().engine_cmd_status);
    em.step(nullptr, nullptr, &in, 100);
    EXPECT_FALSE(em.thrust_cmd().autothrottle_engaged);
    EXPECT_DOUBLE_EQ(0.0, em.status().autothrottle_target);
    EXPECT_NEAR(29.75, em.thrust_cmd().thrust_1, 1e-9) << "the thrust stays where the autothrottle left it";
    l = lever(30);
    em.step(&l, nullptr, &in, 101);
    EXPECT_NEAR(29.75, em.thrust_cmd().thrust_1, 1e-9) << "the lever is where it was: no new command";
    l = lever(40);
    em.step(&l, nullptr, &in, 102);
    EXPECT_DOUBLE_EQ(40.0, em.thrust_cmd().thrust_1) << "until it moves";
}

TEST(EngineManagementTest, CommandsAndTheirOutcomes) {
    EngineManagement em;
    ASSERT_EQ(Status::kOk, em.init());
    const EngEngineIn in = as_in(engines_at(60.0));
    EngFmsControl l = lever(60);
    em.step(&l, nullptr, &in, 1);
    em.command(transfer(12, FuelTank::CENTER, FuelTank::MAIN_LEFT, 1000.0));
    EXPECT_EQ(CmdStatus::FAILED, em.status().engine_cmd_status) << "Main Left is full: nothing could move";

    em.command(set_thrust(1, EngineSelect::ENGINE_2, 40.0));
    em.step(nullptr, nullptr, &in, 2);
    EXPECT_EQ(CmdStatus::ACCEPTED, em.status().engine_cmd_status);
    EXPECT_DOUBLE_EQ(60.0, em.thrust_cmd().thrust_1);
    EXPECT_DOUBLE_EQ(40.0, em.thrust_cmd().thrust_2);
    EXPECT_DOUBLE_EQ(20.0, em.thrust_cmd().asymmetry);
    EXPECT_FALSE(em.thrust_cmd().asymmetric_thrust);
    em.command(set_thrust(2, EngineSelect::ENGINE_1, 90.0));
    em.step(nullptr, nullptr, &in, 3);
    EXPECT_TRUE(em.thrust_cmd().asymmetric_thrust) << "50 points apart";
    EXPECT_TRUE(em.status().asymmetric_thrust);
    em.command(set_thrust(3, EngineSelect::ALL, 70.0));
    em.step(nullptr, nullptr, &in, 4);
    EXPECT_DOUBLE_EQ(70.0, em.thrust_cmd().thrust_1);
    EXPECT_DOUBLE_EQ(70.0, em.thrust_cmd().thrust_2);
    em.command(set_thrust(4, EngineSelect::ALL, 150.0));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, em.status().engine_cmd_status);
    em.step(nullptr, nullptr, &in, 5);
    EXPECT_DOUBLE_EQ(70.0, em.thrust_cmd().thrust_1) << "out of range: not applied";
    em.command(set_thrust(4, EngineSelect::ALL, 20.0));
    em.step(nullptr, nullptr, &in, 6);
    EXPECT_DOUBLE_EQ(70.0, em.thrust_cmd().thrust_1) << "the same cmd_seq is not a new command";
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, em.status().engine_cmd_status);
    em.command(simple(5, EngineCommand::NONE));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, em.status().engine_cmd_status);
    em.command(autothrottle(6, 0.0));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, em.status().engine_cmd_status);
    EXPECT_FALSE(em.thrust().autothrottle);

    // The fuel pass at frame 5 burned the 60 % engines' 2520 kg/h for 0.1 s; now 100 frames at 3600 kg/h per
    // engine: 20 passes of 0.1 kg each, two thirds of an engine's share from its main tank.
    const double pass_at_60 = 2520.0 / 3600.0 * 0.1;
    const double burned_per_engine = pass_at_60 + 20.0 * 0.1;
    const EngEngineIn burning = as_in(engines_at(3600.0 / 42.0));
    for (std::uint32_t frame = 7; frame <= 106; ++frame) {
        em.step(nullptr, nullptr, &burning, frame);
    }
    EXPECT_NEAR(10000.0 - burned_per_engine * 2.0 / 3.0, em.fuel_status().main_left, 1e-6);
    EXPECT_NEAR(40000.0 - 2.0 * burned_per_engine, em.fuel_status().total, 1e-6);
    EXPECT_NEAR(7200.0, em.fuel_status().burn_rate, 1e-3);
    em.command(transfer(8, FuelTank::CENTER, FuelTank::MAIN_LEFT, 1000.0));
    EXPECT_EQ(CmdStatus::ACCEPTED, em.status().engine_cmd_status);
    em.step(nullptr, nullptr, &burning, 107);
    EXPECT_NEAR(10000.0, em.fuel_status().main_left, 1e-6) << "topped up to capacity";
    EXPECT_NEAR(burned_per_engine * 2.0 / 3.0, em.fuel_status().transferred, 1e-6);
    EXPECT_NEAR(40000.0 - 2.0 * burned_per_engine, em.fuel_status().total, 1e-6) << "a transfer keeps the fuel";
    EXPECT_FALSE(em.status().leak_suspected);
    EXPECT_FALSE(em.status().low_fuel);
    em.command(transfer(9, FuelTank::CENTER, FuelTank::CENTER, 10.0));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, em.status().engine_cmd_status);
    em.command(transfer(10, FuelTank::CENTER, FuelTank::MAIN_RIGHT, 0.0));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, em.status().engine_cmd_status);
    em.command(transfer(11, FuelTank::NONE, FuelTank::MAIN_RIGHT, 10.0));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, em.status().engine_cmd_status);
}

TEST(EngineManagementTest, TheEngineStatusCarriesTheAssessment) {
    EngineManagement em;
    ASSERT_EQ(Status::kOk, em.init());
    PlantEngine p = engines_at(100.0);
    p.egt_1 = 800.1;
    p.oil_temperature_1 = 130.1;
    p.n2_1 = 99.0;
    p.n1_1 = 100.0;
    p.oil_pressure_2 = 39.9;
    p.vibration_2 = 0.81;
    p.n1_2 = 99.9;
    p.n2_2 = 99.9;
    p.egt_2 = 800.0;
    p.fuel_flow_2 = 4100.0;
    EngEngineIn in = as_in(p);
    em.step(nullptr, nullptr, &in, 1);
    const EngEngineStatus& e = em.engine_status();
    EXPECT_TRUE(e.egt_high_1);
    EXPECT_TRUE(e.oil_hot_1);
    EXPECT_TRUE(e.overspeed_1) << "N1 at 100";
    EXPECT_FALSE(e.oil_low_1);
    EXPECT_FALSE(e.vibration_high_1);
    EXPECT_FALSE(e.egt_high_2) << "800 is not above 800";
    EXPECT_TRUE(e.oil_low_2);
    EXPECT_TRUE(e.vibration_high_2);
    EXPECT_FALSE(e.overspeed_2);
    EXPECT_FALSE(e.oil_hot_2);
    EXPECT_DOUBLE_EQ(4200.0 + 4100.0, e.total_fuel_flow);
    EXPECT_DOUBLE_EQ(800.1, e.egt_1);
    EXPECT_DOUBLE_EQ(39.9, e.oil_pressure_2);

    p.engine_2_ssm = Ssm::NO_COMPUTED_DATA;
    in = as_in(p);
    em.step(nullptr, nullptr, &in, 2);
    EXPECT_EQ(Ssm::NORMAL, em.engine_status().engine_1_ssm);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, em.engine_status().engine_2_ssm);
    EXPECT_FALSE(em.engine_status().oil_low_2) << "no assessment of an invalid engine";
    EXPECT_DOUBLE_EQ(0.0, em.engine_status().oil_pressure_2);
    EXPECT_DOUBLE_EQ(4200.0, em.engine_status().total_fuel_flow) << "only the valid engine's flow";
    EXPECT_EQ(TerminalState::NORMAL, em.state());

    em.start_self_test();
    EXPECT_TRUE(em.bit().in_progress);
    for (std::uint32_t frame = 3; frame < 14; ++frame) {
        em.step(nullptr, nullptr, &in, frame);
    }
    EXPECT_FALSE(em.bit().in_progress) << "self test done after ten frames, nothing fails";
    EXPECT_TRUE(em.status().healthy);
}

}  // namespace
