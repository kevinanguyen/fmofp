// The terrain-following radar against the Python it replaces: tfr_processor's
// clearance manager and path optimiser (the worked example below was run
// through the frozen Python), tfr_radar's terrain warnings, and the state
// machine, cadence and command handling the ICD defines.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>

#include <fmofp/radar/terrain_following.hpp>

namespace {

using fmofp::Status;
using namespace fmofp::radar;  // NOLINT: test
using namespace fmofp::icd;    // NOLINT: test

TerrainProfile three_samples() {
    // (0 m, 900 m), (101 m, 1600 m), (202 m, 650 m): the example the Python was run on.
    TerrainProfile p{};
    p.samples = 3;
    p.range_m = 202.0;
    p.seq = 7;
    p.elevation_m[0] = 900.0;
    p.elevation_m[1] = 1600.0;
    p.elevation_m[2] = 650.0;
    return p;
}

// tfr_radar._initialize_terrain_data without its noise: 100 samples over 10 km.
TerrainProfile python_hills() {
    TerrainProfile p{};
    p.samples = kMaxProfileSamples;
    p.range_m = 10000.0;
    p.seq = 1;
    for (std::size_t i = 0; i < p.samples; ++i) {
        const double d = p.distance_m(i);
        p.elevation_m[i] = 1000.0 + 500.0 * std::sin(d / 1000.0) + 200.0 * std::sin(d / 300.0);
    }
    return p;
}

PlantTerrain as_block(const TerrainProfile& p) {
    PlantTerrain t{};
    t.terrain_seq = p.seq;
    t.range_m = p.range_m;
    t.elevation_count = static_cast<std::uint16_t>(p.samples);
    for (std::size_t i = 0; i < p.samples; ++i) {
        t.elevation[i].elevation = p.elevation_m[i];
    }
    return t;
}

RdrTfrSensors sensors(double altitude_ft, double airspeed_kt = 486.0) {
    RdrTfrSensors s{};
    s.payload.altitude_ssm = Ssm::NORMAL;
    s.payload.airspeed_ssm = Ssm::NORMAL;
    s.payload.altitude = altitude_ft;
    s.payload.airspeed = airspeed_kt;
    return s;
}

RdrTfrModeCmd command(std::uint8_t seq, TerminalState state, TfrMode mode, double set_clearance_ft = 984.0) {
    RdrTfrModeCmd c{};
    c.cmd_seq = seq;
    c.requested_state = state;
    c.requested_mode = mode;
    c.set_clearance = set_clearance_ft;
    return c;
}

TEST(TerrainProfile, DistancesAreNumpyLinspace) {
    const TerrainProfile p = python_hills();
    EXPECT_DOUBLE_EQ(0.0, p.distance_m(0));
    EXPECT_DOUBLE_EQ(10000.0 / 99.0, p.distance_m(1));
    EXPECT_DOUBLE_EQ(50.0 * (10000.0 / 99.0), p.distance_m(50));
    EXPECT_DOUBLE_EQ(10000.0, p.distance_m(99)) << "the last sample is exactly the range";
    TerrainProfile one{};
    one.samples = 1;
    one.range_m = 500.0;
    EXPECT_DOUBLE_EQ(0.0, one.distance_m(0));
}

TEST(Clearance, LevelsAtTheThresholds) {
    EXPECT_EQ(ClearanceLevel::CLEAR, clearance_level(600.0));
    EXPECT_EQ(ClearanceLevel::LOW, clearance_level(599.9));
    EXPECT_EQ(ClearanceLevel::LOW, clearance_level(300.0));
    EXPECT_EQ(ClearanceLevel::CAUTION, clearance_level(299.9));
    EXPECT_EQ(ClearanceLevel::CAUTION, clearance_level(150.0));
    EXPECT_EQ(ClearanceLevel::PULL_UP, clearance_level(149.9));
    EXPECT_EQ(ClearanceLevel::PULL_UP, clearance_level(-600.0));
}

TEST(Clearance, AssessesTheExampleAsThePythonDid) {
    ClearanceAssessment a{};
    assess_profile(three_samples(), 1000.0, ClearanceLevel::CLEAR, a);
    EXPECT_EQ(ClearanceLevel::PULL_UP, a.level[0]);  // 100 m
    EXPECT_EQ(ClearanceLevel::PULL_UP, a.level[1]);  // -600 m
    EXPECT_EQ(ClearanceLevel::LOW, a.level[2]);      // 350 m
    EXPECT_DOUBLE_EQ(-600.0, a.min_clearance_m);
    EXPECT_EQ(1U, a.min_index);
    EXPECT_EQ(3U, a.advisories);
    EXPECT_EQ(ClearanceLevel::PULL_UP, a.master);
}

TEST(Clearance, MasterLevelLatchesUntilAClearProfile) {
    TerrainProfile low = three_samples();
    low.elevation_m[0] = 600.0;  // 400 m: LOW
    low.elevation_m[1] = 650.0;  // 350 m: LOW
    low.elevation_m[2] = 300.0;  // 700 m: CLEAR
    ClearanceAssessment a{};
    assess_profile(low, 1000.0, ClearanceLevel::PULL_UP, a);
    EXPECT_EQ(ClearanceLevel::PULL_UP, a.master) << "advisories exist, so the worse level held stays";
    EXPECT_EQ(2U, a.advisories);
    assess_profile(low, 1000.0, ClearanceLevel::CLEAR, a);
    EXPECT_EQ(ClearanceLevel::LOW, a.master) << "from CLEAR it rises to the worst seen";
    TerrainProfile clear = low;
    clear.elevation_m[0] = 0.0;
    clear.elevation_m[1] = 0.0;
    assess_profile(clear, 1000.0, ClearanceLevel::PULL_UP, a);
    EXPECT_EQ(ClearanceLevel::CLEAR, a.master) << "a profile with no advisory resets the latch";
    EXPECT_EQ(0U, a.advisories);
    EXPECT_DOUBLE_EQ(700.0, a.min_clearance_m);
    EXPECT_EQ(2U, a.min_index);
    assess_profile(clear, 1000.0, ClearanceLevel::NONE, a);
    EXPECT_EQ(ClearanceLevel::CLEAR, a.master) << "NONE is no latch at all";
}

TEST(Path, ReproducesThePythonsCommands) {
    // PathOptimiser.compute_path([(0, 900), (101, 1600), (202, 650)], 1000) at 250 m/s with 300 m clearance:
    // sample 0 cannot move (no time), sample 1 may move 10.1 m (not a new command), sample 2 may move 20.2 m
    // down towards 950 and does, then the final CLEARANCE command clears the 1600 m peak.
    PathCommands c{};
    compute_path(three_samples(), 1000.0, kDefaultSpeedMps, 300.0, c);
    ASSERT_EQ(3U, c.count);
    EXPECT_DOUBLE_EQ(0.0, c.command[0].distance_m);
    EXPECT_DOUBLE_EQ(1000.0, c.command[0].altitude_m);
    EXPECT_EQ(PathReason::TERRAIN_AVOIDANCE, c.command[0].reason);
    EXPECT_DOUBLE_EQ(202.0, c.command[1].distance_m);
    EXPECT_NEAR(979.8, c.command[1].altitude_m, 1e-9);
    EXPECT_EQ(PathReason::TERRAIN_AVOIDANCE, c.command[1].reason);
    EXPECT_DOUBLE_EQ(202.0, c.command[2].distance_m);
    EXPECT_DOUBLE_EQ(1900.0, c.command[2].altitude_m);
    EXPECT_EQ(PathReason::CLEARANCE, c.command[2].reason);
}

TEST(Path, EmptyProfileAndASingleSampleWindow) {
    PathCommands c{};
    compute_path(TerrainProfile{}, 1000.0, 250.0, 300.0, c);
    EXPECT_EQ(0U, c.count);
    // Sample 0 is always at 0 m, so the window is never empty and the Python's
    // "first twenty" fallback never runs; with the rest of the samples beyond
    // the look-ahead the window is that one sample, which can only command the
    // current altitude (no distance, no time to change it).
    TerrainProfile far{};
    far.samples = 30;
    far.range_m = 29.0 * 20000.0;
    for (std::size_t i = 0; i < far.samples; ++i) {
        far.elevation_m[i] = 500.0;
    }
    far.elevation_m[0] = 0.0;
    compute_path(far, 1000.0, 250.0, 300.0, c);
    ASSERT_EQ(1U, c.count);
    EXPECT_DOUBLE_EQ(1000.0, c.command[0].altitude_m);
    EXPECT_EQ(PathReason::TERRAIN_AVOIDANCE, c.command[0].reason);
    // Below the required altitude the final CLEARANCE command is added.
    compute_path(far, 100.0, 250.0, 300.0, c);
    ASSERT_EQ(2U, c.count);
    EXPECT_DOUBLE_EQ(100.0, c.command[0].altitude_m);
    EXPECT_DOUBLE_EQ(300.0, c.command[1].altitude_m);
    EXPECT_DOUBLE_EQ(0.0, c.command[1].distance_m);
    EXPECT_EQ(PathReason::CLEARANCE, c.command[1].reason);
}

TEST(Path, HillsGiveAClimbingSmoothPath) {
    PathCommands c{};
    compute_path(python_hills(), 1000.0, kDefaultSpeedMps, 300.0, c);
    ASSERT_GT(c.count, 2U);
    ASSERT_LE(c.count, kMaxPathCommands);
    double max_terrain = 0.0;
    const TerrainProfile hills = python_hills();
    for (std::size_t i = 0; i < hills.samples; ++i) {
        if (hills.distance_m(i) <= kLookaheadM) {
            max_terrain = std::fmax(max_terrain, hills.elevation_m[i]);
        }
    }
    for (std::size_t i = 1; i < c.count; ++i) {
        EXPECT_LE(c.command[i - 1].distance_m, c.command[i].distance_m) << "nearest first";
        if (c.command[i].reason == PathReason::TERRAIN_AVOIDANCE) {
            EXPECT_GT(std::fabs(c.command[i].altitude_m - c.command[i - 1].altitude_m), kCommandStepM - 1e-9);
        }
    }
    EXPECT_EQ(PathReason::CLEARANCE, c.command[c.count - 1].reason);
    EXPECT_DOUBLE_EQ(max_terrain + 300.0, c.command[c.count - 1].altitude_m);
}

TEST(Warnings, HighAndSteepTerrain) {
    EXPECT_EQ(TerrainWarning::HIGH_TERRAIN, terrain_warning(1600.0));
    EXPECT_EQ(TerrainWarning::STEEP_TERRAIN, terrain_warning(1500.0)) << "1500 is not above 1500, but far from 1000";
    EXPECT_EQ(TerrainWarning::STEEP_TERRAIN, terrain_warning(650.0));
    EXPECT_EQ(TerrainWarning::STEEP_TERRAIN, terrain_warning(1301.0));
    EXPECT_EQ(TerrainWarning::NONE, terrain_warning(1300.0));
    EXPECT_EQ(TerrainWarning::NONE, terrain_warning(700.0));
    EXPECT_EQ(TerrainWarning::STEEP_TERRAIN, terrain_warning(699.0));
}

TEST(TerrainFollowing, StartsReadyWithoutGuidance) {
    auto tfr = std::make_unique<TerrainFollowing>();
    ASSERT_EQ(Status::kOk, tfr->init());
    EXPECT_EQ(TerminalState::INITIALIZING, tfr->state());
    EXPECT_TRUE(tfr->take_products_update()) << "the empty block is published first";
    EXPECT_EQ(TfrMode::NONE, tfr->products().mode);
    tfr->step(nullptr, nullptr, 1);
    EXPECT_EQ(TerminalState::NORMAL, tfr->state());
    EXPECT_EQ(TfrMode::NONE, tfr->mode());
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, tfr->guidance().clearance_ssm);
    EXPECT_DOUBLE_EQ(kDefaultSetClearanceFt, tfr->guidance().set_clearance);
    EXPECT_FALSE(tfr->take_products_update());
}

TEST(TerrainFollowing, AssessesEachProfileAndPublishesTheBlockAtItsCadence) {
    auto tfr = std::make_unique<TerrainFollowing>();
    ASSERT_EQ(Status::kOk, tfr->init());
    (void)tfr->take_products_update();
    tfr->step(nullptr, nullptr, 1);
    tfr->command_mode(command(1, TerminalState::NORMAL, TfrMode::TERRAIN_FOLLOWING));
    EXPECT_EQ(TfrMode::TERRAIN_FOLLOWING, tfr->mode());
    EXPECT_FALSE(tfr->status().cmd_rejected);
    EXPECT_DOUBLE_EQ(984.0, tfr->set_clearance_ft());
    const RdrTfrSensors s = sensors(3280.84);  // 1000 m, give or take the conversion
    const double alt_m = 3280.84 * kMPerFt;
    tfr->step(&s, nullptr, 2);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, tfr->guidance().clearance_ssm) << "no profile yet";
    EXPECT_TRUE(tfr->status().mode_transition);
    const PlantTerrain block = as_block(three_samples());
    tfr->step(&s, &block, 3);
    const RdrTfrGuidance& g = tfr->guidance();
    EXPECT_EQ(Ssm::NORMAL, g.clearance_ssm);
    EXPECT_EQ(ClearanceLevel::PULL_UP, g.level);
    EXPECT_TRUE(g.pull_up);
    EXPECT_EQ(TfrMode::TERRAIN_FOLLOWING, g.mode);
    EXPECT_EQ(7U, g.profile_seq);
    EXPECT_NEAR((alt_m - 900.0) * kFtPerM, g.terrain_clearance, 1e-9);
    EXPECT_NEAR((alt_m - 1600.0) * kFtPerM, g.min_clearance, 1e-9);
    EXPECT_DOUBLE_EQ(101.0, g.min_clearance_distance);
    EXPECT_TRUE(g.high_terrain);
    EXPECT_TRUE(g.steep_terrain);
    EXPECT_EQ(2U, g.warning_count);
    EXPECT_TRUE(g.commands_valid);
    EXPECT_NEAR(alt_m * kFtPerM, g.commanded_altitude, 1e-9);
    EXPECT_DOUBLE_EQ(0.0, g.command_distance);
    EXPECT_FALSE(tfr->status().mode_transition);
    EXPECT_EQ(1U, tfr->profiles_assessed());

    ASSERT_TRUE(tfr->take_products_update());
    const RdrTfrElevation& p = tfr->products();
    EXPECT_EQ(TfrMode::TERRAIN_FOLLOWING, p.mode);
    EXPECT_EQ(3U, p.profile_count);
    EXPECT_EQ(3U, p.commands_count);
    EXPECT_DOUBLE_EQ(1600.0, p.profile[1].elevation);
    EXPECT_EQ(ClearanceLevel::PULL_UP, p.profile[1].level);
    EXPECT_EQ(TerrainWarning::HIGH_TERRAIN, p.profile[1].warning);
    EXPECT_EQ(TerrainWarning::STEEP_TERRAIN, p.profile[2].warning);
    EXPECT_EQ(PathReason::CLEARANCE, p.commands[2].reason);
    EXPECT_FALSE(p.commands_truncated);
    EXPECT_DOUBLE_EQ(202.0, p.range_m);
    EXPECT_DOUBLE_EQ(984.0, p.set_clearance);

    // A second profile five frames later is assessed (guidance) but not republished (block).
    PlantTerrain next = block;
    next.terrain_seq = 8;
    tfr->step(&s, &next, 8);
    EXPECT_EQ(8U, tfr->guidance().profile_seq);
    EXPECT_FALSE(tfr->take_products_update());
    EXPECT_EQ(7U, tfr->products().profile_seq);
    // Fifteen frames after the last block, the next profile is published.
    next.terrain_seq = 9;
    tfr->step(&s, &next, 3 + kElevationBlockFrames);
    EXPECT_TRUE(tfr->take_products_update());
    EXPECT_EQ(9U, tfr->products().profile_seq);
    EXPECT_EQ(3U, tfr->profiles_assessed());

    // The block holds the whole 100-sample profile and encodes within its maximum.
    const PlantTerrain hills = as_block(python_hills());
    tfr->step(&s, &hills, 60);
    ASSERT_TRUE(tfr->take_products_update());
    EXPECT_EQ(kMaxProfileSamples, tfr->products().profile_count);
    std::array<fmofp::bus::Word, RdrTfrElevation::kMaxPayloadWords> w{};
    std::uint16_t length = 0;
    const Status encoded = tfr->products().encode(w.data(), static_cast<std::uint16_t>(w.size()), length);
    EXPECT_TRUE(encoded == Status::kOk || encoded == Status::kClamped);
    EXPECT_LE(length, RdrTfrElevation::kMaxPayloadWords);
    RdrTfrGuidance::Words gw{};
    EXPECT_TRUE(fmofp::ok(tfr->guidance().encode(gw)));
}

TEST(TerrainFollowing, StaleInputsWithdrawTheGuidance) {
    auto tfr = std::make_unique<TerrainFollowing>();
    ASSERT_EQ(Status::kOk, tfr->init());
    tfr->step(nullptr, nullptr, 1);
    tfr->command_mode(command(1, TerminalState::NORMAL, TfrMode::TFR_SEARCH));
    const RdrTfrSensors s = sensors(5000.0);
    const PlantTerrain block = as_block(three_samples());
    tfr->step(&s, &block, 2);
    EXPECT_EQ(Ssm::NORMAL, tfr->guidance().clearance_ssm);
    tfr->step(nullptr, nullptr, 3);
    tfr->step(nullptr, nullptr, 4);
    EXPECT_EQ(Ssm::NORMAL, tfr->guidance().clearance_ssm) << "sensors are good for three frames";
    tfr->step(nullptr, nullptr, 2 + kSensorStaleFrames);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, tfr->guidance().clearance_ssm) << "then stale";
    EXPECT_EQ(TfrMode::TFR_SEARCH, tfr->guidance().mode) << "the mode is still reported";
    tfr->step(&s, nullptr, 6);
    EXPECT_EQ(Ssm::NORMAL, tfr->guidance().clearance_ssm) << "the profile is still fresh";
    for (std::uint32_t f = 7; f <= 2 + kTerrainStaleFrames; ++f) {
        tfr->step(&s, nullptr, f);
    }
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, tfr->guidance().clearance_ssm) << "until the profile ages out";
    RdrTfrSensors bad = s;
    bad.payload.altitude_ssm = Ssm::NO_COMPUTED_DATA;
    tfr->step(&bad, &block, 30);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, tfr->guidance().clearance_ssm) << "no altitude, no assessment";
}

TEST(TerrainFollowing, StandbyClearsAndCommandsAreChecked) {
    auto tfr = std::make_unique<TerrainFollowing>();
    ASSERT_EQ(Status::kOk, tfr->init());
    (void)tfr->take_products_update();
    tfr->step(nullptr, nullptr, 1);
    tfr->command_mode(command(1, TerminalState::NORMAL, TfrMode::TFR_ACTIVE, 10.0));
    EXPECT_DOUBLE_EQ(kMinSetClearanceFt, tfr->set_clearance_ft()) << "clamped to the ICD minimum";
    tfr->command_mode(command(2, TerminalState::NORMAL, TfrMode::TFR_ACTIVE, 9000.0));
    EXPECT_DOUBLE_EQ(kMaxSetClearanceFt, tfr->set_clearance_ft()) << "clamped to the ICD maximum";
    tfr->command_mode(command(2, TerminalState::NORMAL, TfrMode::TFR_ACTIVE, 500.0));
    EXPECT_DOUBLE_EQ(kMaxSetClearanceFt, tfr->set_clearance_ft()) << "a repeated cmd_seq is ignored";
    tfr->command_mode(command(3, TerminalState::DEGRADED, TfrMode::TFR_ACTIVE, 500.0));
    EXPECT_TRUE(tfr->status().cmd_rejected) << "DEGRADED cannot be commanded";
    EXPECT_EQ(3U, tfr->status().last_cmd_seq);
    EXPECT_EQ(TerminalState::NORMAL, tfr->state());
    const RdrTfrSensors s = sensors(5000.0);
    const PlantTerrain block = as_block(three_samples());
    tfr->step(&s, &block, 2);
    ASSERT_TRUE(tfr->take_products_update());
    EXPECT_EQ(TfrMode::TFR_ACTIVE, tfr->products().mode);
    tfr->command_mode(command(4, TerminalState::STANDBY, TfrMode::TFR_ACTIVE, 500.0));
    EXPECT_FALSE(tfr->status().cmd_rejected);
    tfr->step(&s, &block, 3);
    EXPECT_EQ(TerminalState::STANDBY, tfr->state());
    EXPECT_EQ(TfrMode::NONE, tfr->status().mode);
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, tfr->guidance().clearance_ssm);
    ASSERT_TRUE(tfr->take_products_update()) << "the cleared block is published once";
    EXPECT_EQ(TfrMode::NONE, tfr->products().mode);
    EXPECT_EQ(0U, tfr->products().profile_count);
    EXPECT_DOUBLE_EQ(500.0, tfr->products().set_clearance);
    tfr->step(&s, &block, 4);
    EXPECT_FALSE(tfr->take_products_update());
    // Back to work: the latch starts clean and the first profile is published at once.
    tfr->command_mode(command(5, TerminalState::NORMAL, TfrMode::TERRAIN_FOLLOWING, 500.0));
    tfr->step(&s, &block, 5);
    EXPECT_EQ(Ssm::NORMAL, tfr->guidance().clearance_ssm);
    EXPECT_TRUE(tfr->take_products_update());
    tfr->reset();
    EXPECT_EQ(TerminalState::INITIALIZING, tfr->state());
    EXPECT_DOUBLE_EQ(kDefaultSetClearanceFt, tfr->set_clearance_ft());
    tfr->start_self_test();
    EXPECT_TRUE(tfr->bit().in_progress);
    for (std::uint32_t f = 1; f <= 12; ++f) {
        tfr->step(nullptr, nullptr, f);
    }
    EXPECT_FALSE(tfr->bit().in_progress);
    EXPECT_TRUE(tfr->status().healthy);
}

}  // namespace
