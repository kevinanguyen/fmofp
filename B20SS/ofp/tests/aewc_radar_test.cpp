// The AEWC radar: its radar equation, sectors, classification, priorities and electronic protection, and the
// track table and block over the plant's traffic.
#include <array>
#include <cmath>

#include <fmofp/radar/aewc_radar.hpp>
#include <gtest/gtest.h>

namespace {

namespace radar = fmofp::radar;
namespace icd = fmofp::icd;
using fmofp::Status;
using icd::AewcMode;
using icd::EcmMitigation;
using icd::JamType;
using icd::TargetClass;
using icd::TerminalState;

constexpr double kPi = 3.14159265358979323846;

struct Aircraft {
    double east_m;
    double north_m;
    double up_m;
    double ve;
    double vn;
    double vu;
    double rcs;
};

icd::PlantTraffic traffic_of(std::initializer_list<Aircraft> aircraft, std::uint8_t seq = 1) {
    icd::PlantTraffic t{};
    t.traffic_seq = seq;
    std::uint16_t n = 0;
    for (const Aircraft& a : aircraft) {
        icd::PlantTrafficTarget& rec = t.target[n];
        const double horizontal = std::hypot(a.east_m, a.north_m);
        rec.target_id = static_cast<std::uint8_t>(n + 1U);
        rec.azimuth = std::fmod(std::atan2(a.east_m, a.north_m) * 180.0 / kPi + 360.0, 360.0);
        rec.range = std::sqrt(horizontal * horizontal + a.up_m * a.up_m) / 1000.0;
        rec.elevation = std::atan2(a.up_m, horizontal) * 180.0 / kPi;
        rec.velocity_east = a.ve;
        rec.velocity_north = a.vn;
        rec.velocity_up = a.vu;
        rec.rcs = a.rcs;
        ++n;
    }
    t.target_count = n;
    return t;
}

icd::RdrAewcModeCmd command(std::uint8_t seq, TerminalState state, AewcMode mode, double start = 0.0, double end = 90.0) {
    icd::RdrAewcModeCmd c{};
    c.cmd_seq = seq;
    c.requested_state = state;
    c.requested_mode = mode;
    c.azimuth_start = start;
    c.azimuth_end = end;
    c.elevation = 0.0;
    return c;
}

TEST(AewcProcessing, TheRadarEquationReadAsThePythonReadIt) {
    // 100 kW, 40 dB gain squared, a 3 cm wavelength, 10 m2 at 5 km: 7.26e-8 W, which the Python took for -71.4 dBm
    // against its -110 dBm floor.
    EXPECT_NEAR(38.61, radar::aewc_snr_db(10.0, 5000.0), 0.01);
    EXPECT_NEAR(14.53, radar::aewc_snr_db(10.0, 20000.0), 0.01);
    EXPECT_DOUBLE_EQ(0.0, radar::aewc_snr_db(10.0, 100000.0)) << "negative SNRs are clipped to 0: most of the picture";
    EXPECT_DOUBLE_EQ(0.0, radar::aewc_snr_db(0.0, 5000.0)) << "no cross-section, no echo";
    EXPECT_NEAR(15.6, radar::aewc_snr_db(0.05, 5000.0), 0.1) << "a stealth aircraft close in";
}

TEST(AewcProcessing, SectorsCountFromEastCounterClockwiseWithinTheElevationBand) {
    EXPECT_EQ(1U, radar::aewc_sector_of({1000.0, 0.0, 0.0}));
    EXPECT_EQ(2U, radar::aewc_sector_of({0.0, 1000.0, 0.0}));
    EXPECT_EQ(4U, radar::aewc_sector_of({-1000.0, 0.0, 0.0}));
    EXPECT_EQ(5U, radar::aewc_sector_of({0.0, -1000.0, 0.0}));
    EXPECT_EQ(6U, radar::aewc_sector_of({1000.0, -1.0, 0.0})) << "359.94 degrees";
    EXPECT_EQ(1U, radar::aewc_sector_of({1000.0, 0.0, -200.0})) << "-11 degrees of elevation is inside";
    EXPECT_EQ(0U, radar::aewc_sector_of({1000.0, 0.0, -300.0})) << "-17 degrees is below the band";
    EXPECT_EQ(0U, radar::aewc_sector_of({1000.0, 0.0, 1500.0})) << "56 degrees is above it";
}

TEST(AewcProcessing, TheClassificationRule) {
    EXPECT_EQ(TargetClass::STEALTH, radar::aewc_classify(450.0, 0.0, true));
    EXPECT_EQ(TargetClass::FIGHTER, radar::aewc_classify(401.0, 0.0, false));
    EXPECT_EQ(TargetClass::HIGH_ALT, radar::aewc_classify(100.0, 30001.0, false));
    EXPECT_EQ(TargetClass::HIGH_ALT, radar::aewc_classify(100.0, -30001.0, false)) << "the Python's abs(z)";
    EXPECT_EQ(TargetClass::UNKNOWN, radar::aewc_classify(400.0, 30000.0, false));
}

TEST(AewcProcessing, TheElectronicProtectionGradesTheNoiseExceedance) {
    radar::EcmAssessment a = radar::ecm_assess(5.9);
    EXPECT_FALSE(a.jammed);
    EXPECT_EQ((JamType::NONE), a.jam_type);
    a = radar::ecm_assess(6.0);
    EXPECT_TRUE(a.jammed);
    EXPECT_NEAR(0.15, a.severity, 1e-12);
    EXPECT_EQ((JamType::DECEPTIVE), a.jam_type);
    EXPECT_EQ((EcmMitigation::FREQ_AGILITY), a.mitigation);
    a = radar::ecm_assess(12.0);
    EXPECT_EQ((JamType::SWEEP), a.jam_type);
    a = radar::ecm_assess(25.0);
    EXPECT_EQ((JamType::SPOT), a.jam_type);
    EXPECT_EQ((EcmMitigation::BEAM_NOTCH), a.mitigation);
    a = radar::ecm_assess(35.0);
    EXPECT_EQ((JamType::CONTINUOUS), a.jam_type);
    EXPECT_EQ((EcmMitigation::POWER), a.mitigation);
    a = radar::ecm_assess(50.0);
    EXPECT_DOUBLE_EQ(1.0, a.severity);
    EXPECT_EQ((EcmMitigation::SKIP), a.mitigation);
}

TEST(AewcProcessing, SectorPrioritiesWeighTracksAgeAndJamming) {
    const std::array<std::uint8_t, 6> tracks{2, 0, 0, 0, 0, 0};
    const std::array<std::uint8_t, 6> hostile{};
    std::array<double, 6> ages{};
    ages.fill(10.0);
    radar::SectorPriorities p = radar::aewc_priorities(tracks, hostile, ages, 0);
    EXPECT_DOUBLE_EQ(7.0, p.score[0]);
    EXPECT_DOUBLE_EQ(5.0, p.score[1]);
    EXPECT_DOUBLE_EQ(7.0 / 32.0, p.dwell[0]);
    EXPECT_DOUBLE_EQ(5.0 / 32.0, p.dwell[5]);
    EXPECT_EQ(0U, p.order[0]);
    EXPECT_EQ(1U, p.order[1]);
    // A jammed sector gets 2 points: sector 3 ties with sector 1 and stays behind it (a stable sort).
    p = radar::aewc_priorities(tracks, hostile, ages, 0x04);
    EXPECT_DOUBLE_EQ(7.0, p.score[2]);
    EXPECT_EQ(0U, p.order[0]);
    EXPECT_EQ(2U, p.order[1]);
    EXPECT_EQ(1U, p.order[2]);
    // Nothing anywhere: `total or 1.0`, and every sector keeps the 5 % floor.
    const std::array<std::uint8_t, 6> none{};
    const std::array<double, 6> fresh{};
    p = radar::aewc_priorities(none, none, fresh, 0);
    EXPECT_DOUBLE_EQ(0.05, p.dwell[3]);
    EXPECT_EQ(0U, p.order[0]);
}

class AewcRadarTest : public ::testing::Test {
protected:
    void SetUp() override { ASSERT_EQ(Status::kOk, radar_.init()); }

    void run(std::uint32_t frames, const icd::PlantTraffic* traffic = nullptr) {
        for (std::uint32_t i = 0; i < frames; ++i) {
            ++frame_;
            radar_.step(i == 0 ? traffic : nullptr, frame_);
        }
    }

    radar::AewcRadar radar_;
    std::uint32_t frame_{0};
};

TEST_F(AewcRadarTest, StandbyHoldsNoTracksAndTheFirstStepMakesItNormal) {
    const icd::PlantTraffic t = traffic_of({{100000.0, 0.0, 0.0, 0.0, 200.0, 0.0, 20.0}});
    run(1, &t);
    EXPECT_EQ(TerminalState::NORMAL, radar_.state());
    EXPECT_EQ(AewcMode::NONE, radar_.mode());
    EXPECT_EQ(0U, radar_.held_tracks());
    EXPECT_TRUE(radar_.take_tracks_update()) << "the empty block is published first";
    EXPECT_EQ(0U, radar_.tracks_block().track_count);
    EXPECT_EQ(6U, radar_.tracks_block().sector_count) << "the six sectors are always reported";
    EXPECT_EQ(AewcMode::NONE, radar_.tracks_block().mode);
}

TEST_F(AewcRadarTest, SearchTracksEverythingWithinFourHundredKilometres) {
    radar_.command_mode(command(1, TerminalState::NORMAL, AewcMode::AEWC_SEARCH, -30.0, 60.0));
    EXPECT_FALSE(radar_.status().cmd_rejected);
    const icd::PlantTraffic t = traffic_of({{20000.0, 0.0, 0.0, 0.0, 420.0, 0.0, 10.0},      // 20 km east, fast
                                            {0.0, 300000.0, 35000.0, 0.0, -100.0, 0.0, 50.0},  // 300 km north, high
                                            {5000.0, 0.0, 0.0, 250.0, 0.0, 0.0, 0.05},       // 5 km: stealth, fast
                                            {0.0, -450000.0, 0.0, 0.0, 0.0, 0.0, 30.0}});     // beyond range
    run(1, &t);
    ASSERT_EQ(3U, radar_.held_tracks());
    EXPECT_EQ(1U, radar_.stealth_tracks());
    const radar::AewcTrack* one = radar_.find(1);
    const radar::AewcTrack* two = radar_.find(2);
    const radar::AewcTrack* three = radar_.find(3);
    ASSERT_NE(nullptr, one);
    ASSERT_NE(nullptr, two);
    ASSERT_NE(nullptr, three);
    EXPECT_NEAR(20000.0, one->position[0], 0.5);
    EXPECT_NEAR(14.53, one->snr_db, 0.02);
    EXPECT_NEAR(14.53 / 30.0, one->confidence, 0.001);
    EXPECT_EQ(TargetClass::FIGHTER, one->classification);
    EXPECT_TRUE(one->detected);
    EXPECT_EQ(1U, one->sector) << "due east";
    EXPECT_EQ(TargetClass::HIGH_ALT, two->classification);
    EXPECT_DOUBLE_EQ(0.0, two->snr_db) << "300 km away: no echo in the Python's arithmetic";
    EXPECT_EQ(2U, two->sector) << "due north, 6.7 degrees up";
    EXPECT_TRUE(three->stealth);
    EXPECT_EQ(TargetClass::STEALTH, three->classification);
    EXPECT_NEAR(15.6, three->snr_db, 0.1);
    EXPECT_EQ(three->detected, three->sector != 0) << "a missed stealth detection leaves the sector counts";
    EXPECT_EQ(icd::Threat::HOSTILE, three->threat) << "small and faster than 200 m/s is the fusion's hostile";
    EXPECT_EQ(3U, radar_.status().track_count);
    EXPECT_EQ(1U, radar_.status().stealth_tracks);
    EXPECT_NE(0U, radar_.status().priority_sector);
    EXPECT_TRUE(radar_.take_tracks_update());
    const icd::RdrAewcTracks& block = radar_.tracks_block();
    EXPECT_EQ(3U, block.track_count);
    EXPECT_EQ(1U, block.track[0].track_id);
    EXPECT_EQ(AewcMode::AEWC_SEARCH, block.mode);
    EXPECT_DOUBLE_EQ(-30.0, block.scan_azimuth_start);
    EXPECT_DOUBLE_EQ(60.0, block.scan_azimuth_end);
    EXPECT_DOUBLE_EQ(0.0, block.sector[0].azimuth_start);
    EXPECT_DOUBLE_EQ(300.0, block.sector[5].azimuth_start);
    EXPECT_EQ(1U, block.sector[0].active_tracks);
    EXPECT_EQ(1U, block.sector[1].active_tracks);
    EXPECT_GT(block.sector[0].priority_score, block.sector[2].priority_score) << "a sector with tracks ranks higher";
    EXPECT_GE(block.sector[2].dwell_fraction, 0.05);
}

TEST_F(AewcRadarTest, TheScanProgressAdvancesATenthASecondAndStandbyClearsEverything) {
    radar_.command_mode(command(1, TerminalState::NORMAL, AewcMode::AEWC_SURVEILLANCE));
    const icd::PlantTraffic t = traffic_of({{20000.0, 0.0, 0.0, 0.0, 100.0, 0.0, 10.0}});
    run(1, &t);
    EXPECT_DOUBLE_EQ(0.0, radar_.sector(0).scan_progress);
    run(50);
    EXPECT_NEAR(0.1, radar_.sector(0).scan_progress, 1e-12);
    run(50);
    EXPECT_NEAR(0.2, radar_.sector(3).scan_progress, 1e-12);
    radar_.command_mode(command(2, TerminalState::STANDBY, AewcMode::NONE));
    run(1);
    EXPECT_EQ(0U, radar_.held_tracks());
    EXPECT_DOUBLE_EQ(0.0, radar_.sector(0).scan_progress);
    EXPECT_EQ(AewcMode::NONE, radar_.status().mode);
    EXPECT_EQ(0U, radar_.status().track_count);
}

TEST_F(AewcRadarTest, TracksAreContinuedByProximityAndDroppedWhenUnseen) {
    radar_.command_mode(command(1, TerminalState::NORMAL, AewcMode::AEWC_SEARCH));
    icd::PlantTraffic t = traffic_of({{100000.0, 0.0, 0.0, 0.0, 400.0, 0.0, 20.0}, {0.0, 200000.0, 0.0, 0.0, -100.0, 0.0, 20.0}});
    run(1, &t);
    ASSERT_EQ(2U, radar_.held_tracks());
    t = traffic_of({{0.0, 199980.0, 0.0, 0.0, -100.0, 0.0, 20.0}, {100000.0, 80.0, 0.0, 0.0, 400.0, 0.0, 20.0}}, 2);
    run(10, &t);  // frames 2..11, traffic at 2
    ASSERT_EQ(2U, radar_.held_tracks());
    EXPECT_NEAR(80.0, radar_.find(1)->position[1], 0.5);
    EXPECT_NEAR(199980.0, radar_.find(2)->position[1], 0.5);
    EXPECT_EQ(2U, radar_.updates());
    run(6);  // frame 17: 15 frames unseen, still held
    EXPECT_EQ(2U, radar_.held_tracks());
    run(1);  // frame 18: dropped
    EXPECT_EQ(0U, radar_.held_tracks());
}

TEST_F(AewcRadarTest, TheBlockIsRepublishedNoMoreOftenThanEveryThirtyFiveFrames) {
    radar_.command_mode(command(1, TerminalState::NORMAL, AewcMode::AEWC_TRACK));
    icd::PlantTraffic t = traffic_of({{50000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0}});
    run(1, &t);
    EXPECT_TRUE(radar_.take_tracks_update());
    t = traffic_of({{50000.0, 10.0, 0.0, 0.0, 0.0, 0.0, 20.0}}, 2);
    run(10, &t);  // frames 2..11, the aircraft seen at 2
    EXPECT_FALSE(radar_.take_tracks_update()) << "changed, but too soon";
    run(10, &t);  // seen again at 12 and 22: the track is kept
    run(10, &t);
    EXPECT_FALSE(radar_.take_tracks_update()) << "frame 31: still too soon";
    run(5, &t);  // frame 36: 35 frames after the first publication
    EXPECT_TRUE(radar_.take_tracks_update());
    EXPECT_NEAR(10.0, radar_.tracks_block().track[0].y, 0.5);
    EXPECT_EQ(AewcMode::AEWC_TRACK, radar_.tracks_block().mode);
}

TEST_F(AewcRadarTest, CommandsFollowTheSequenceRuleAndBadOnesAreRejected) {
    radar_.command_mode(command(1, TerminalState::NORMAL, AewcMode::SECTOR_SCAN));
    radar_.command_mode(command(1, TerminalState::STANDBY, AewcMode::NONE));
    EXPECT_EQ(AewcMode::SECTOR_SCAN, radar_.mode()) << "a repeated cmd_seq is ignored";
    radar_.command_mode(command(2, TerminalState::FAILURE, AewcMode::AEWC_SEARCH));
    EXPECT_TRUE(radar_.status().cmd_rejected);
    EXPECT_EQ(TerminalState::NORMAL, radar_.state());
    radar_.command_mode(command(3, TerminalState::NORMAL, static_cast<AewcMode>(12)));
    EXPECT_TRUE(radar_.status().cmd_rejected);
    radar_.command_mode(command(4, TerminalState::NORMAL, AewcMode::STEALTH_DETECTION));
    EXPECT_FALSE(radar_.status().cmd_rejected);
    EXPECT_EQ(AewcMode::STEALTH_DETECTION, radar_.mode());
    EXPECT_EQ(4U, radar_.status().last_cmd_seq);
}

}  // namespace
