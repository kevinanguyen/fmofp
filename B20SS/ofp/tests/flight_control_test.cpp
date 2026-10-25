// The FCS logic against the Python it replaces: every branch of
// groundCollisionAvoidanceSys._evaluate and performaneMonitoring._check_envelope,
// plus the state machine, staleness, and command handling the ICD defines.
#include <gtest/gtest.h>

#include <cstdint>

#include <fmofp/fcs/flight_control.hpp>

namespace {

using fmofp::Status;
using fmofp::fcs::EngineFuelFlow;
using fmofp::fcs::FlightControl;
using fmofp::fcs::GcasThresholds;
using fmofp::fcs::TerrainClearance;
using fmofp::fcs::kSensorStaleFrames;
using namespace fmofp::icd;  // NOLINT: test

PlantSensors sensors(double altitude, double vs) {
    PlantSensors s{};
    s.altitude_ssm = Ssm::NORMAL;
    s.airspeed_ssm = Ssm::NORMAL;
    s.pitch_ssm = Ssm::NORMAL;
    s.roll_ssm = Ssm::NORMAL;
    s.heading_ssm = Ssm::NORMAL;
    s.vertical_speed_ssm = Ssm::NORMAL;
    s.aoa_ssm = Ssm::NORMAL;
    s.g_ssm = Ssm::NORMAL;
    s.flaps_ssm = Ssm::NORMAL;
    s.gear = GearPosition::UP;
    s.altitude = altitude;
    s.airspeed = 300.0;
    s.vertical_speed = vs;
    s.g_force = 1.0;
    s.angle_of_attack = 2.0;
    s.heading = 90.0;
    return s;
}

FcsSensors as_fcs(const PlantSensors& p) {
    FcsSensors f{};
    f.payload = p;
    return f;
}

// The TFR's minimum clearance ahead, as FCS_TERRAIN carries it.
TerrainClearance clearance(double feet) { return TerrainClearance{true, feet}; }
const TerrainClearance kNoClearance{};
const EngineFuelFlow kNoFuel{};

// The engine computer's status as BCAST_ENG_ENGINE_STATUS carries it, both engines valid.
BcastEngEngineStatus as_engine(double fuel_flow_1, double fuel_flow_2) {
    BcastEngEngineStatus e{};
    e.payload.engine_1_ssm = Ssm::NORMAL;
    e.payload.engine_2_ssm = Ssm::NORMAL;
    e.payload.fuel_flow_1 = fuel_flow_1;
    e.payload.fuel_flow_2 = fuel_flow_2;
    e.payload.total_fuel_flow = fuel_flow_1 + fuel_flow_2;
    return e;
}

FcsTerrain as_terrain(double feet) {
    FcsTerrain t{};
    t.payload.clearance_ssm = Ssm::NORMAL;
    t.payload.level = ClearanceLevel::CLEAR;
    t.payload.min_clearance = feet;
    t.payload.terrain_clearance = feet;
    return t;
}

struct Expected {
    bool pullup, sink_rate, terrain, alt_low;
    GcasStatus status;
};

void expect_gcas(const PlantSensors& s, const TerrainClearance& c, const Expected& e, const char* why) {
    const FcsGcasAlerts a = FlightControl::evaluate_gcas(s, c, GcasThresholds{});
    EXPECT_EQ(e.pullup, a.pullup) << why;
    EXPECT_EQ(e.sink_rate, a.sink_rate) << why;
    EXPECT_EQ(e.terrain, a.terrain) << why;
    EXPECT_EQ(e.alt_low, a.alt_low) << why;
    EXPECT_EQ(e.status, a.status) << why;
}

TEST(Gcas, ReproducesEveryBranchOfThePython) {
    // (altitude, vs, clearance) -> alerts, from groundCollisionAvoidanceSys._evaluate.
    const TerrainClearance high = clearance(10000);
    expect_gcas(sensors(30000, 0), high, {false, false, false, false, GcasStatus::ARMED}, "cruise");
    expect_gcas(sensors(1500, -4000), high, {true, false, false, false, GcasStatus::WARNING}, "pullup at the threshold");
    expect_gcas(sensors(1999, -4500), high, {true, false, false, false, GcasStatus::WARNING}, "pullup below 2000 ft");
    expect_gcas(sensors(2000, -4500), high, {false, true, false, false, GcasStatus::WARNING},
                "at 2000 ft it is only a sink rate");
    expect_gcas(sensors(30000, -2000), high, {false, true, false, false, GcasStatus::WARNING},
                "sink rate at the threshold");
    expect_gcas(sensors(30000, -1999), high, {false, false, false, false, GcasStatus::ARMED}, "just above sink rate");
    expect_gcas(sensors(30000, 0), clearance(499), {false, false, true, false, GcasStatus::WARNING},
                "terrain below 500 ft");
    expect_gcas(sensors(30000, 0), clearance(500), {false, false, false, false, GcasStatus::ARMED},
                "500 ft is not terrain");
    expect_gcas(sensors(30000, 0), clearance(-650), {false, false, true, false, GcasStatus::WARNING},
                "terrain ahead above the aircraft is terrain");
    expect_gcas(sensors(199, 0), high, {false, false, false, true, GcasStatus::CAUTION}, "alt low");
    expect_gcas(sensors(200, 0), high, {false, false, false, false, GcasStatus::ARMED}, "200 ft is not low");
    expect_gcas(sensors(150, 0), clearance(100), {false, false, true, false, GcasStatus::WARNING},
                "terrain suppresses alt low");
    expect_gcas(sensors(150, -4000), clearance(100), {true, false, true, false, GcasStatus::WARNING},
                "pullup and terrain together");
    expect_gcas(sensors(150, -3000), high, {false, true, false, true, GcasStatus::WARNING},
                "sink rate and alt low: warning wins");
}

TEST(Gcas, InvalidSensorsBehaveLikeThePythonsDefaults) {
    // The Python read altitude as 30000 and vs as 0 when the FMS had none,
    // and clearance as None when the TFR had none.
    PlantSensors s = sensors(100, -5000);
    s.altitude_ssm = Ssm::NO_COMPUTED_DATA;
    FcsGcasAlerts a = FlightControl::evaluate_gcas(s, clearance(100), GcasThresholds{});
    EXPECT_FALSE(a.pullup) << "no altitude: cannot be below 2000 ft";
    EXPECT_TRUE(a.sink_rate) << "the vs alone still says sink rate";
    EXPECT_FALSE(a.alt_low);
    EXPECT_TRUE(a.terrain);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, a.altitude_ssm);
    EXPECT_DOUBLE_EQ(0.0, a.altitude);

    s = sensors(100, -5000);
    s.vertical_speed_ssm = Ssm::FAILURE_WARNING;
    a = FlightControl::evaluate_gcas(s, clearance(100), GcasThresholds{});
    EXPECT_FALSE(a.pullup);
    EXPECT_FALSE(a.sink_rate);
    EXPECT_TRUE(a.terrain);

    s = sensors(100, 0);
    a = FlightControl::evaluate_gcas(s, kNoClearance, GcasThresholds{});
    EXPECT_FALSE(a.terrain) << "no clearance: no terrain alert";
    EXPECT_TRUE(a.alt_low) << "and nothing suppresses alt low";
    EXPECT_EQ(GcasStatus::CAUTION, a.status);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, a.clearance_ssm);
}

TEST(Gcas, TerrainComesFromTheRadarAndGoesStale) {
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    const FcsSensors in = as_fcs(sensors(3000, 0));
    const FcsTerrain terrain = as_terrain(400);
    fc.step(&in, nullptr, nullptr, 1);
    EXPECT_FALSE(fc.gcas().terrain) << "no FCS_TERRAIN yet";
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.gcas().clearance_ssm);
    fc.step(&in, &terrain, nullptr, 2);
    EXPECT_TRUE(fc.gcas().terrain);
    EXPECT_EQ(Ssm::NORMAL, fc.gcas().clearance_ssm);
    EXPECT_DOUBLE_EQ(400.0, fc.gcas().terrain_clearance);
    // 10 Hz: the clearance holds for three periods, then it is gone.
    for (std::uint32_t f = 3; f < 2 + fmofp::fcs::kTerrainStaleFrames; ++f) {
        fc.step(&in, nullptr, nullptr, f);
        EXPECT_TRUE(fc.gcas().terrain) << "frame " << f;
    }
    fc.step(&in, nullptr, nullptr, 2 + fmofp::fcs::kTerrainStaleFrames);
    EXPECT_FALSE(fc.gcas().terrain);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.gcas().clearance_ssm);
    // A radar that is not operating says so in the SSM: no clearance either.
    FcsTerrain standby = as_terrain(400);
    standby.payload.clearance_ssm = Ssm::NO_COMPUTED_DATA;
    fc.step(&in, &standby, nullptr, 40);
    EXPECT_FALSE(fc.gcas().terrain);
}

TEST(Envelope, ExceedancesAtTheLimits) {
    PlantSensors s = sensors(30000, 0);
    s.g_force = 9.0;
    s.angle_of_attack = 25.0;
    s.roll = -60.0;
    s.pitch = 45.0;
    s.vertical_speed = -6000.0;
    FcsPerfStatus p = FlightControl::evaluate_envelope(s, kNoFuel, {});
    EXPECT_FALSE(p.exceed_g || p.exceed_aoa || p.exceed_roll || p.exceed_pitch || p.exceed_vertical_speed)
        << "a value at its limit is not an exceedance (strict >)";
    s.g_force = 9.01;
    s.angle_of_attack = -25.1;
    s.roll = 60.1;
    s.pitch = -45.1;
    s.vertical_speed = 6001.0;
    p = FlightControl::evaluate_envelope(s, kNoFuel, {});
    EXPECT_TRUE(p.exceed_g && p.exceed_aoa && p.exceed_roll && p.exceed_pitch && p.exceed_vertical_speed);
    EXPECT_EQ(Ssm::NORMAL, p.g_ssm);
    EXPECT_DOUBLE_EQ(9.01, p.g_force);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, p.fuel_ssm) << "no fuel flow given";

    s.g_ssm = Ssm::NO_COMPUTED_DATA;
    p = FlightControl::evaluate_envelope(s, kNoFuel, {});
    EXPECT_FALSE(p.exceed_g) << "an invalid value reads as 0, as the Python's dict.get did";
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, p.g_ssm);
}

TEST(FlightControlTest, FuelEfficiencyComesFromTheEngineComputer) {
    PlantSensors s = sensors(30000.0, 0.0);
    s.airspeed = 450.0;
    // performaneMonitoring._compute_fuel_efficiency: 450 kt over 6720 kg/h is 450 / 112 nm per kg.
    FcsPerfStatus p = FlightControl::evaluate_envelope(s, EngineFuelFlow{true, 6720.0}, {});
    EXPECT_EQ(Ssm::NORMAL, p.fuel_ssm);
    EXPECT_NEAR(450.0 / 112.0, p.fuel_efficiency, 1e-9);
    p = FlightControl::evaluate_envelope(s, EngineFuelFlow{true, 0.0}, {});
    EXPECT_EQ(Ssm::NORMAL, p.fuel_ssm);
    EXPECT_DOUBLE_EQ(0.0, p.fuel_efficiency) << "the Python returned 0.0 for a flow that is not positive";
    p = FlightControl::evaluate_envelope(s, EngineFuelFlow{true, 1.0}, {});
    EXPECT_DOUBLE_EQ(fmofp::fcs::kMaxFuelEfficiency, p.fuel_efficiency) << "held to what the word can carry";
    s.airspeed_ssm = Ssm::NO_COMPUTED_DATA;
    p = FlightControl::evaluate_envelope(s, EngineFuelFlow{true, 6720.0}, {});
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, p.fuel_ssm) << "no airspeed, no efficiency";

    // Through the partition: the broadcast status feeds it, and it goes stale after three periods.
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    const FcsSensors in = as_fcs(sensors(30000.0, 0.0));
    const BcastEngEngineStatus engine = as_engine(3360.0, 3360.0);
    fc.step(&in, nullptr, &engine, 1);
    EXPECT_EQ(Ssm::NORMAL, fc.perf().fuel_ssm);
    EXPECT_NEAR(300.0 / 112.0, fc.perf().fuel_efficiency, 1e-9) << "the sensors' 300 kt over 6720 kg/h";
    for (std::uint32_t frame = 2; frame < 1 + fmofp::fcs::kEngineStaleFrames; ++frame) {
        fc.step(&in, nullptr, nullptr, frame);
    }
    EXPECT_EQ(Ssm::NORMAL, fc.perf().fuel_ssm) << "still within three periods";
    fc.step(&in, nullptr, nullptr, 1 + fmofp::fcs::kEngineStaleFrames);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.perf().fuel_ssm) << "the engine status went stale";
    EXPECT_DOUBLE_EQ(0.0, fc.perf().fuel_efficiency);
    BcastEngEngineStatus one_engine = as_engine(3360.0, 0.0);
    one_engine.payload.engine_2_ssm = Ssm::NO_COMPUTED_DATA;
    fc.step(&in, nullptr, &one_engine, 100);
    EXPECT_EQ(Ssm::NORMAL, fc.perf().fuel_ssm) << "one valid engine is a flow";
    EXPECT_NEAR(300.0 / 56.0, fc.perf().fuel_efficiency, 1e-9);
    BcastEngEngineStatus no_engine = as_engine(0.0, 0.0);
    no_engine.payload.engine_1_ssm = Ssm::NO_COMPUTED_DATA;
    no_engine.payload.engine_2_ssm = Ssm::NO_COMPUTED_DATA;
    fc.step(&in, nullptr, &no_engine, 101);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.perf().fuel_ssm) << "no valid engine, no flow";
}

TEST(FlightControl, StateFollowsTheSensors) {
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    EXPECT_EQ(TerminalState::INITIALIZING, fc.state());
    fc.step(nullptr, nullptr, nullptr, 1);
    EXPECT_EQ(TerminalState::INITIALIZING, fc.state());
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.attitude().roll_ssm);
    EXPECT_EQ(GcasStatus::OFF, fc.gcas().status);

    const FcsSensors in = as_fcs(sensors(30000, 0));
    fc.step(&in, nullptr, nullptr, 2);
    EXPECT_EQ(TerminalState::NORMAL, fc.state());
    EXPECT_EQ(Ssm::NORMAL, fc.attitude().roll_ssm);
    EXPECT_EQ(GcasStatus::ARMED, fc.gcas().status);
    EXPECT_DOUBLE_EQ(90.0, fc.attitude().yaw);

    // Two frames without sensors are still fresh; the third is stale.
    fc.step(nullptr, nullptr, nullptr, 3);
    fc.step(nullptr, nullptr, nullptr, 4);
    EXPECT_EQ(TerminalState::NORMAL, fc.state());
    fc.step(nullptr, nullptr, nullptr, 2 + kSensorStaleFrames);
    EXPECT_EQ(TerminalState::DEGRADED, fc.state());
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.attitude().pitch_ssm);
    EXPECT_EQ(GcasStatus::OFF, fc.gcas().status);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.gcas().altitude_ssm);

    fc.step(&in, nullptr, nullptr, 10);
    EXPECT_EQ(TerminalState::NORMAL, fc.state()) << "recovers when data returns";
}

TEST(FlightControl, YawWrapsHeadingIntoPlusMinus180) {
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    PlantSensors p = sensors(30000, 0);
    p.heading = 270.0;
    FcsSensors in = as_fcs(p);
    fc.step(&in, nullptr, nullptr, 1);
    EXPECT_DOUBLE_EQ(-90.0, fc.attitude().yaw);
    p.heading = 180.0;
    in = as_fcs(p);
    fc.step(&in, nullptr, nullptr, 2);
    EXPECT_DOUBLE_EQ(180.0, fc.attitude().yaw) << "exactly 180 stays 180, which the field cannot carry";
}

TEST(FlightControl, ModeCommandsFollowTheSequenceRule) {
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    const FcsSensors in = as_fcs(sensors(30000, 0));
    fc.step(&in, nullptr, nullptr, 1);
    ASSERT_EQ(TerminalState::NORMAL, fc.state());

    FcsModeCmd cmd{};
    cmd.requested_state = TerminalState::STANDBY;
    cmd.cmd_seq = 7;
    fc.command_mode(cmd);
    fc.step(&in, nullptr, nullptr, 2);
    EXPECT_EQ(TerminalState::STANDBY, fc.state());
    EXPECT_EQ(7, fc.status().last_cmd_seq);
    EXPECT_FALSE(fc.status().cmd_rejected);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.attitude().roll_ssm) << "standby publishes nothing computed";

    cmd.requested_state = TerminalState::NORMAL;
    fc.command_mode(cmd);  // same cmd_seq: ignored
    fc.step(&in, nullptr, nullptr, 3);
    EXPECT_EQ(TerminalState::STANDBY, fc.state());
    cmd.cmd_seq = 8;
    fc.command_mode(cmd);
    fc.step(&in, nullptr, nullptr, 4);
    EXPECT_EQ(TerminalState::NORMAL, fc.state());

    cmd.requested_state = TerminalState::FAILURE;  // not something the bus may ask for
    cmd.cmd_seq = 9;
    fc.command_mode(cmd);
    fc.step(&in, nullptr, nullptr, 5);
    EXPECT_EQ(TerminalState::NORMAL, fc.state());
    EXPECT_TRUE(fc.status().cmd_rejected);
    EXPECT_EQ(9, fc.status().last_cmd_seq);
}

TEST(FlightControl, SurfaceCommandsAreClampedLikeFmsControl) {
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    EXPECT_EQ(Ssm::NORMAL, fc.surfaces().elevator_ssm) << "surfaces start known, at neutral";
    EXPECT_DOUBLE_EQ(0.0, fc.surfaces().elevator);

    FcsSurfaceCmd cmd{};
    cmd.surface = ControlSurface::ELEVATOR;
    cmd.cmd_seq = 1;
    cmd.position = -12.5;
    fc.command_surface(cmd);
    EXPECT_DOUBLE_EQ(-12.5, fc.surfaces().elevator);
    EXPECT_EQ(CmdStatus::COMPLETED, fc.surfaces().last_cmd_status);
    EXPECT_EQ(1, fc.surfaces().last_cmd_seq);

    cmd.surface = ControlSurface::FLAPS;
    cmd.cmd_seq = 2;
    cmd.position = -5.0;  // flaps cannot go negative
    fc.command_surface(cmd);
    EXPECT_DOUBLE_EQ(0.0, fc.surfaces().flaps);
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, fc.surfaces().last_cmd_status);

    cmd.surface = ControlSurface::AILERONS;
    cmd.cmd_seq = 3;
    cmd.position = 150.0;
    fc.command_surface(cmd);
    EXPECT_DOUBLE_EQ(100.0, fc.surfaces().ailerons) << "limited to full travel";
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, fc.surfaces().last_cmd_status);

    cmd.surface = ControlSurface::NONE;
    cmd.cmd_seq = 4;
    cmd.position = 1.0;
    fc.command_surface(cmd);
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, fc.surfaces().last_cmd_status);

    cmd.cmd_seq = 4;  // repeat: ignored, status unchanged
    cmd.surface = ControlSurface::RUDDER;
    fc.command_surface(cmd);
    EXPECT_DOUBLE_EQ(0.0, fc.surfaces().rudder);
}

TEST(FlightControl, SelfTestRunsForAFewFramesAndResetReinitialises) {
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    const FcsSensors in = as_fcs(sensors(30000, 0));
    fc.step(&in, nullptr, nullptr, 1);
    fc.start_self_test();
    fc.step(&in, nullptr, nullptr, 2);
    EXPECT_TRUE(fc.bit().in_progress);
    for (std::uint32_t f = 3; f < 20; ++f) {
        fc.step(&in, nullptr, nullptr, f);
    }
    EXPECT_FALSE(fc.bit().in_progress);
    EXPECT_FALSE(fc.bit().failed);
    EXPECT_TRUE(fc.status().healthy);

    fc.reset();
    EXPECT_EQ(TerminalState::INITIALIZING, fc.state());
    EXPECT_EQ(Ssm::FAILURE_WARNING, fc.attitude().roll_ssm) << "fresh outputs are the all-zero fail-safe";
    fc.step(nullptr, nullptr, nullptr, 1);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, fc.attitude().roll_ssm) << "and NCD once it has run without data";
}

TEST(FlightControl, OutputsEncodeCleanly) {
    FlightControl fc;
    ASSERT_EQ(Status::kOk, fc.init());
    const FcsSensors in = as_fcs(sensors(1500, -4500));
    const FcsTerrain terrain = as_terrain(300);
    fc.step(&in, &terrain, nullptr, 1);
    FcsGcasAlerts::Words gw{};
    EXPECT_EQ(Status::kOk, fc.gcas().encode(gw));
    FcsGcasAlerts back{};
    ASSERT_EQ(Status::kOk, FcsGcasAlerts::decode(gw, back));
    EXPECT_TRUE(back.pullup);
    EXPECT_TRUE(back.terrain);
    EXPECT_EQ(GcasStatus::WARNING, back.status);
    EXPECT_DOUBLE_EQ(-4500.0, back.vertical_speed);
    FcsAttitude::Words aw{};
    EXPECT_EQ(Status::kOk, fc.attitude().encode(aw));
    FcsStatus::Words sw{};
    EXPECT_EQ(Status::kOk, fc.status().encode(sw));
    FcsPerfStatus::Words pw{};
    EXPECT_EQ(Status::kOk, fc.perf().encode(pw));
    FcsSurfaceStatus::Words fw{};
    EXPECT_EQ(Status::kOk, fc.surfaces().encode(fw));
}

}  // namespace
