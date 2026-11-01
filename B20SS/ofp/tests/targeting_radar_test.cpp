// The targeting radar: tracks from the plant's traffic, the lock, the filter, the block.
#include <array>
#include <cmath>

#include <fmofp/radar/targeting_radar.hpp>
#include <gtest/gtest.h>

namespace {

namespace radar = fmofp::radar;
namespace icd = fmofp::icd;
using fmofp::Status;
using icd::LockStatus;
using icd::TargetingMode;
using icd::TerminalState;
using icd::Threat;
using icd::TrackFilter;

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

// PLANT_TRAFFIC as the plant would publish it for aircraft at these positions relative to the ownship.
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

icd::RdrTgtModeCmd command(std::uint8_t seq, TerminalState state, TargetingMode mode, TrackFilter filter = TrackFilter::ALL,
                           std::uint16_t lock = 0) {
    icd::RdrTgtModeCmd c{};
    c.cmd_seq = seq;
    c.requested_state = state;
    c.requested_mode = mode;
    c.track_filter = filter;
    c.lock_track_id = lock;
    return c;
}

class TargetingRadarTest : public ::testing::Test {
protected:
    void SetUp() override { ASSERT_EQ(Status::kOk, radar_.init()); }

    void run(std::uint32_t frames, const icd::PlantTraffic* traffic = nullptr) {
        for (std::uint32_t i = 0; i < frames; ++i) {
            ++frame_;
            radar_.step(i == 0 ? traffic : nullptr, frame_);
        }
    }

    radar::TargetingRadar radar_;
    std::uint32_t frame_{0};
};

TEST_F(TargetingRadarTest, StandbyHoldsNoTracksAndTheFirstStepMakesItNormal) {
    const icd::PlantTraffic t = traffic_of({{10000.0, 0.0, 0.0, 0.0, 200.0, 0.0, 2.0}});
    run(1, &t);
    EXPECT_EQ(TerminalState::NORMAL, radar_.state());
    EXPECT_EQ(TargetingMode::NONE, radar_.mode());
    EXPECT_EQ(0U, radar_.held_tracks());
    EXPECT_TRUE(radar_.take_tracks_update()) << "the empty block is published first";
    EXPECT_EQ(0U, radar_.tracks_block().track_count);
    EXPECT_EQ(TargetingMode::NONE, radar_.tracks_block().mode);
}

TEST_F(TargetingRadarTest, SearchMakesATrackPerDetectionWithinRange) {
    radar_.command_mode(command(1, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH));
    EXPECT_FALSE(radar_.status().cmd_rejected);
    const icd::PlantTraffic t = traffic_of({{30000.0, 40000.0, 6000.0, 0.0, 300.0, 0.0, 2.0},   // 50 km, fast, high
                                            {-20000.0, 0.0, -100.0, 50.0, 0.0, 0.0, 40.0},      // 20 km
                                            {80000.0, 80000.0, 0.0, 0.0, 0.0, 0.0, 5.0}});      // 113 km: beyond range
    run(1, &t);
    ASSERT_EQ(2U, radar_.held_tracks());
    const radar::TgtTrack* one = radar_.find(1);
    const radar::TgtTrack* two = radar_.find(2);
    ASSERT_NE(nullptr, one);
    ASSERT_NE(nullptr, two);
    EXPECT_NEAR(30000.0, one->position[0], 0.5);
    EXPECT_NEAR(40000.0, one->position[1], 0.5);
    EXPECT_NEAR(6000.0, one->position[2], 0.5);
    EXPECT_DOUBLE_EQ(300.0, one->velocity[1]);
    const double range_1 = std::sqrt(30000.0 * 30000.0 + 40000.0 * 40000.0 + 6000.0 * 6000.0);
    EXPECT_NEAR(30.0 - 20.0 * std::log10(range_1 / 1000.0), one->snr_db, 1e-6);
    EXPECT_NEAR(std::fmax(0.0, std::fmin(1.0, one->snr_db / 30.0)), one->confidence, 1e-12);
    EXPECT_EQ(icd::TargetClass::FIGHTER, one->classification) << "faster than 250 m/s above 5000 m, whatever the draw";
    EXPECT_EQ(Threat::HOSTILE, one->threat) << "2 m2 and 300 m/s";
    EXPECT_EQ(icd::IffIdentity::UNKNOWN, one->identity);
    EXPECT_NEAR(-20000.0, two->position[0], 0.5);
    EXPECT_EQ(Threat::UNKNOWN, two->threat) << "40 m2";
    EXPECT_TRUE(two->stealth.seen);
    EXPECT_NEAR(0.05 * 0.2 / (0.05 * 0.2 + 0.95), two->stealth_probability, 1e-9) << "a 40 m2 draw is no stealth";
    EXPECT_EQ(2U, radar_.status().track_count);
    EXPECT_EQ(LockStatus::NONE, radar_.status().lock_status);
    EXPECT_TRUE(radar_.take_tracks_update());
    EXPECT_EQ(2U, radar_.tracks_block().track_count);
    EXPECT_EQ(1U, radar_.tracks_block().track[0].track_id);
    EXPECT_EQ(TargetingMode::TARGET_SEARCH, radar_.tracks_block().mode);
    EXPECT_EQ(1U, radar_.tracks_block().update_seq);
}

TEST_F(TargetingRadarTest, TracksAreContinuedByProximityAndDroppedWhenUnseen) {
    radar_.command_mode(command(1, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH));
    icd::PlantTraffic t = traffic_of({{10000.0, 0.0, 0.0, 0.0, 200.0, 0.0, 2.0}, {0.0, 30000.0, 0.0, 0.0, -100.0, 0.0, 2.0}});
    run(1, &t);
    ASSERT_EQ(2U, radar_.held_tracks());
    // Ten frames on, each moved as its velocity says, in the other order: the ids follow the aircraft.
    t = traffic_of({{0.0, 30000.0 - 20.0, 0.0, 0.0, -100.0, 0.0, 2.0}, {10000.0, 40.0, 0.0, 0.0, 200.0, 0.0, 2.0}}, 2);
    run(10, &t);
    ASSERT_EQ(2U, radar_.held_tracks());
    ASSERT_NE(nullptr, radar_.find(1));
    ASSERT_NE(nullptr, radar_.find(2));
    EXPECT_NEAR(40.0, radar_.find(1)->position[1], 0.5);
    EXPECT_NEAR(29980.0, radar_.find(2)->position[1], 0.5);
    EXPECT_EQ(2U, radar_.updates());
    // A detection far from every prediction is a new track; a track unseen for more than 15 frames is dropped.
    t = traffic_of({{10000.0, 80.0, 0.0, 0.0, 200.0, 0.0, 2.0}, {-50000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0}}, 3);
    run(1, &t);  // frame 12: track 2 was last seen at frame 2
    EXPECT_EQ(3U, radar_.held_tracks()) << "the second aircraft is unseen but not yet dropped";
    EXPECT_NE(nullptr, radar_.find(3));
    run(5);  // frame 17: 15 frames unseen, still held
    EXPECT_EQ(3U, radar_.held_tracks());
    run(1);  // frame 18: dropped
    EXPECT_EQ(2U, radar_.held_tracks());
    EXPECT_EQ(nullptr, radar_.find(2));
    // No traffic at all for a while: every track goes.
    run(10);
    EXPECT_EQ(0U, radar_.held_tracks());
}

TEST_F(TargetingRadarTest, LockOnlyInTrackModeAndOnlyOnAHeldTrack) {
    radar_.command_mode(command(1, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH));
    const icd::PlantTraffic t = traffic_of({{2000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0}, {60000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0}});
    run(1, &t);
    radar_.command_mode(command(2, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH, TrackFilter::ALL, 1));
    run(1);
    EXPECT_EQ(0U, radar_.locked_track()) << "the Python refused lock requests outside TRACK mode";
    radar_.command_mode(command(3, TerminalState::NORMAL, TargetingMode::TARGET_TRACK, TrackFilter::ALL, 1));
    run(1);
    EXPECT_EQ(1U, radar_.locked_track());
    EXPECT_NEAR(std::fmin(1.0, (30.0 - 20.0 * std::log10(2.0)) / 30.0), radar_.lock_quality(), 1e-9);
    EXPECT_EQ(LockStatus::LOCKED, radar_.status().lock_status) << "quality above 0.5 at 2 km";
    EXPECT_EQ(1U, radar_.status().locked_track_id);
    radar_.command_mode(command(4, TerminalState::NORMAL, TargetingMode::TARGET_TRACK, TrackFilter::ALL, 2));
    run(1);
    EXPECT_EQ(2U, radar_.locked_track());
    EXPECT_EQ(LockStatus::ACQUIRING, radar_.status().lock_status) << "at 60 km the quality is below 0.5";
    radar_.command_mode(command(5, TerminalState::NORMAL, TargetingMode::TARGET_TRACK, TrackFilter::ALL, 9));
    run(1);
    EXPECT_EQ(2U, radar_.locked_track()) << "a track the radar does not hold leaves the lock as it is";
    radar_.command_mode(command(6, TerminalState::NORMAL, TargetingMode::TARGET_TRACK, TrackFilter::ALL, 0));
    run(1);
    EXPECT_EQ(0U, radar_.locked_track()) << "0 releases";
    // The lock goes with its track, and with STANDBY.
    radar_.command_mode(command(7, TerminalState::NORMAL, TargetingMode::TARGET_TRACK, TrackFilter::ALL, 1));
    run(1);
    EXPECT_EQ(1U, radar_.locked_track());
    const icd::PlantTraffic other = traffic_of({{60000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0}}, 2);
    run(16, &other);  // frames 8..23: track 1, last seen at frame 1, is dropped at frame 17; track 2 is seen at 8
    EXPECT_EQ(0U, radar_.locked_track()) << "track 1 was not seen again";
    EXPECT_EQ(1U, radar_.held_tracks());
    radar_.command_mode(command(8, TerminalState::NORMAL, TargetingMode::TARGET_TRACK, TrackFilter::ALL, 2));
    run(1, &other);
    EXPECT_EQ(2U, radar_.locked_track());
    radar_.command_mode(command(9, TerminalState::STANDBY, TargetingMode::NONE));
    run(1);
    EXPECT_EQ(0U, radar_.held_tracks());
    EXPECT_EQ(0U, radar_.locked_track());
    EXPECT_EQ(TargetingMode::NONE, radar_.status().mode);
}

TEST_F(TargetingRadarTest, TheFilterSelectsWhatTheBlockReportsByThreat) {
    radar_.command_mode(command(1, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH, TrackFilter::HOSTILE));
    const icd::PlantTraffic t = traffic_of({{10000.0, 0.0, 0.0, 0.0, 300.0, 0.0, 2.0},    // hostile: small and fast
                                            {0.0, 10000.0, 0.0, 0.0, 100.0, 0.0, 2.0},    // slow: unknown
                                            {0.0, -10000.0, 0.0, 0.0, 300.0, 0.0, 40.0}}); // big: unknown
    run(1, &t);
    EXPECT_EQ(3U, radar_.held_tracks()) << "the filter selects the report, not the tracking";
    ASSERT_TRUE(radar_.take_tracks_update());
    ASSERT_EQ(1U, radar_.tracks_block().track_count);
    EXPECT_EQ(1U, radar_.tracks_block().track[0].track_id);
    EXPECT_EQ(Threat::HOSTILE, radar_.tracks_block().track[0].threat);
    radar_.command_mode(command(2, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH, TrackFilter::UNKNOWN));
    run(15);
    ASSERT_TRUE(radar_.take_tracks_update());
    EXPECT_EQ(2U, radar_.tracks_block().track_count);
    EXPECT_EQ(2U, radar_.tracks_block().track[0].track_id);
    EXPECT_EQ(3U, radar_.tracks_block().track[1].track_id);
    EXPECT_EQ(TrackFilter::UNKNOWN, radar_.status().track_filter);
    EXPECT_EQ(3U, radar_.status().track_count);
}

TEST_F(TargetingRadarTest, TheBlockIsRepublishedNoMoreOftenThanEveryFifteenFrames) {
    radar_.command_mode(command(1, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH));
    icd::PlantTraffic t = traffic_of({{10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0}});
    run(1, &t);
    EXPECT_TRUE(radar_.take_tracks_update());
    t = traffic_of({{10000.0, 10.0, 0.0, 0.0, 0.0, 0.0, 2.0}}, 2);
    run(5, &t);
    EXPECT_FALSE(radar_.take_tracks_update()) << "changed, but too soon";
    run(10);
    EXPECT_TRUE(radar_.take_tracks_update());
    EXPECT_NEAR(10.0, radar_.tracks_block().track[0].y, 0.5);
    run(30);
    EXPECT_TRUE(radar_.take_tracks_update()) << "the track was dropped for want of traffic: the block changed";
    EXPECT_EQ(0U, radar_.tracks_block().track_count);
    run(30);
    EXPECT_FALSE(radar_.take_tracks_update()) << "nothing changed";
}

TEST_F(TargetingRadarTest, CommandsFollowTheSequenceRuleAndBadOnesAreRejected) {
    radar_.command_mode(command(1, TerminalState::NORMAL, TargetingMode::TARGET_SEARCH));
    radar_.command_mode(command(1, TerminalState::STANDBY, TargetingMode::NONE));
    EXPECT_EQ(TargetingMode::TARGET_SEARCH, radar_.mode()) << "a repeated cmd_seq is ignored";
    icd::RdrTgtModeCmd bad = command(2, TerminalState::FAILURE, TargetingMode::TARGET_SEARCH);
    radar_.command_mode(bad);
    EXPECT_TRUE(radar_.status().cmd_rejected);
    EXPECT_EQ(TerminalState::NORMAL, radar_.state());
    bad = command(3, TerminalState::NORMAL, static_cast<TargetingMode>(9));
    radar_.command_mode(bad);
    EXPECT_TRUE(radar_.status().cmd_rejected);
    radar_.command_mode(command(4, TerminalState::NORMAL, TargetingMode::GROUND_MAPPING));
    EXPECT_FALSE(radar_.status().cmd_rejected);
    EXPECT_EQ(TargetingMode::GROUND_MAPPING, radar_.mode());
    EXPECT_EQ(4U, radar_.status().last_cmd_seq);
}

}  // namespace
