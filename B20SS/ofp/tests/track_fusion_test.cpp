// The track fusion: clustering within the gate, the merge, the ids, the block.
#include <array>
#include <cmath>

#include <fmofp/radar/track_fusion.hpp>
#include <gtest/gtest.h>

namespace {

namespace radar = fmofp::radar;
namespace icd = fmofp::icd;
using fmofp::Status;
using icd::IffIdentity;
using icd::TargetClass;
using icd::Threat;

radar::TrackReport report(double x, double y, double z, double vx, double vy, double vz, TargetClass cls, double rcs,
                          double weight, std::uint8_t sensor) {
    radar::TrackReport r{};
    r.position = {x, y, z};
    r.velocity = {vx, vy, vz};
    r.classification = cls;
    r.identity = IffIdentity::UNKNOWN;
    r.rcs = rcs;
    r.weight = weight;
    r.sensor = sensor;
    return r;
}

TEST(TrackFusion, OneSensorMakesOneFusedTrackPerReportWithItsWeightAsConfidence) {
    radar::TrackFusion fusion;
    ASSERT_EQ(Status::kOk, fusion.init());
    EXPECT_TRUE(fusion.take_update()) << "the empty block is published first";
    const std::array<radar::TrackReport, 2> reports{
        report(10000.0, 0.0, 0.0, 0.0, 300.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting),
        report(0.0, 20000.0, 500.0, 0.0, 100.0, 0.0, TargetClass::BOMBER, 40.0, 0.8, radar::kSensorTargeting)};
    fusion.epoch(reports.data(), reports.size(), true, false);
    ASSERT_EQ(2U, fusion.count());
    EXPECT_EQ(1U, fusion.track(0).id);
    EXPECT_EQ(2U, fusion.track(1).id);
    EXPECT_DOUBLE_EQ(10000.0, fusion.track(0).position[0]);
    EXPECT_DOUBLE_EQ(300.0, fusion.track(0).velocity[1]);
    EXPECT_NEAR(0.8, fusion.track(0).confidence, 1e-12);
    EXPECT_EQ(Threat::HOSTILE, fusion.track(0).threat) << "2 m2 at 300 m/s";
    EXPECT_EQ(Threat::UNKNOWN, fusion.track(1).threat);
    EXPECT_EQ(radar::kSensorTargeting, fusion.track(0).sources);
    EXPECT_EQ(1U, fusion.hostile_count());
    EXPECT_TRUE(fusion.take_update());
    EXPECT_EQ(2U, fusion.block().track_count);
    EXPECT_EQ(1U, fusion.block().epoch);
    EXPECT_EQ(1U, fusion.block().hostile_count);
    EXPECT_TRUE(fusion.block().targeting_present);
    EXPECT_FALSE(fusion.block().aewc_present);
    EXPECT_TRUE(fusion.block().track[0].from_targeting);
    EXPECT_FALSE(fusion.block().track[0].from_aewc);
}

TEST(TrackFusion, ReportsWithinTheGateMergeByWeightWhateverTheSensor) {
    radar::TrackFusion fusion;
    ASSERT_EQ(Status::kOk, fusion.init());
    // Two targeting tracks 300 m apart: one cluster, as the Python merged them (it never checked the sensor).
    std::array<radar::TrackReport, 3> reports{
        report(10000.0, 0.0, 0.0, 0.0, 300.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting),
        report(10300.0, 0.0, 0.0, 0.0, 300.0, 0.0, TargetClass::BOMBER, 2.0, 0.8, radar::kSensorTargeting),
        report(10000.0, 600.0, 0.0, 0.0, 300.0, 0.0, TargetClass::DRONE, 2.0, 0.8, radar::kSensorTargeting)};
    fusion.epoch(reports.data(), reports.size(), true, false);
    ASSERT_EQ(2U, fusion.count()) << "the third is 600 m from the first and 670 m from the pair's centroid";
    EXPECT_NEAR(10150.0, fusion.track(0).position[0], 1e-9);
    EXPECT_NEAR(1.0 - 0.2 * 0.2, fusion.track(0).confidence, 1e-12);
    EXPECT_EQ(TargetClass::FIGHTER, fusion.track(0).classification) << "equal weights: the first report anchors";
    // The targeting and AEWC views of one aircraft: the union of the weights, the heavier sensor's class.
    ASSERT_EQ(Status::kOk, fusion.init());
    reports = {report(10000.0, 0.0, 0.0, 0.0, 300.0, 0.0, TargetClass::UNKNOWN, 2.0, 0.7, radar::kSensorAewc),
               report(10100.0, 0.0, 0.0, 0.0, 280.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting),
               report(50000.0, 0.0, 0.0, 0.0, 0.0, 0.0, TargetClass::HIGH_ALT, 50.0, 0.7, radar::kSensorAewc)};
    fusion.epoch(reports.data(), reports.size(), true, true);
    ASSERT_EQ(2U, fusion.count());
    const radar::FusedTrack& both = fusion.track(0);
    EXPECT_NEAR(1.0 - 0.2 * 0.3, both.confidence, 1e-12);
    EXPECT_NEAR((10100.0 * 0.8 + 10000.0 * 0.7) / 1.5, both.position[0], 1e-9) << "weighted by sensor";
    EXPECT_NEAR((280.0 * 0.8 + 300.0 * 0.7) / 1.5, both.velocity[1], 1e-9);
    EXPECT_EQ(TargetClass::FIGHTER, both.classification) << "the heavier sensor's";
    EXPECT_EQ(static_cast<std::uint8_t>(radar::kSensorTargeting | radar::kSensorAewc), both.sources);
    EXPECT_EQ(Threat::HOSTILE, both.threat) << "the speed of the heaviest report, 280 m/s, and 2 m2";
    EXPECT_EQ(radar::kSensorAewc, fusion.track(1).sources);
    EXPECT_TRUE(fusion.take_update());
    EXPECT_TRUE(fusion.block().track[0].from_targeting);
    EXPECT_TRUE(fusion.block().track[0].from_aewc);
}

TEST(TrackFusion, IdsCarryOverToTheNearestTrackOfThePreviousEpoch) {
    radar::TrackFusion fusion;
    ASSERT_EQ(Status::kOk, fusion.init());
    std::array<radar::TrackReport, 3> reports{
        report(10000.0, 0.0, 0.0, 100.0, 0.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting),
        report(0.0, 30000.0, 0.0, 0.0, -100.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting),
        report(-40000.0, 0.0, 0.0, 0.0, 0.0, 0.0, TargetClass::BOMBER, 40.0, 0.8, radar::kSensorTargeting)};
    fusion.epoch(reports.data(), reports.size(), true, false);
    ASSERT_EQ(3U, fusion.count());
    // Half a second on: the first moved 50 m, the second 50 m, the third is gone and a new one appears.
    reports = {report(-20000.0, 20000.0, 0.0, 0.0, 0.0, 0.0, TargetClass::DRONE, 0.5, 0.8, radar::kSensorTargeting),
               report(0.0, 29950.0, 0.0, 0.0, -100.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting),
               report(10050.0, 0.0, 0.0, 100.0, 0.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting)};
    fusion.epoch(reports.data(), reports.size(), true, false);
    ASSERT_EQ(3U, fusion.count());
    EXPECT_EQ(4U, fusion.track(0).id) << "new";
    EXPECT_EQ(2U, fusion.track(1).id);
    EXPECT_EQ(1U, fusion.track(2).id);
    EXPECT_EQ(2U, fusion.block().epoch);
    ASSERT_TRUE(fusion.take_update());
    EXPECT_EQ(1U, fusion.block().track[0].track_id) << "the block is in id order";
    EXPECT_EQ(2U, fusion.block().track[1].track_id);
    EXPECT_EQ(4U, fusion.block().track[2].track_id);
    // A previous track is used once: of two new tracks near one old one, the first in cluster order takes its id.
    std::array<radar::TrackReport, 2> pair{
        report(10400.0, 0.0, 0.0, 100.0, 0.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting),
        report(9800.0, 0.0, 0.0, 100.0, 0.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting)};
    fusion.epoch(pair.data(), pair.size(), true, false);
    ASSERT_EQ(2U, fusion.count()) << "600 m apart: two clusters";
    EXPECT_EQ(1U, fusion.track(0).id) << "350 m from the previous track 1";
    EXPECT_EQ(5U, fusion.track(1).id) << "track 1 was taken, nothing else within 500 m: a new id";
}

TEST(TrackFusion, StepRunsAnEpochEveryTwentyFiveFrames) {
    radar::TrackFusion fusion;
    ASSERT_EQ(Status::kOk, fusion.init());
    (void)fusion.take_update();
    const std::array<radar::TrackReport, 1> reports{
        report(10000.0, 0.0, 0.0, 0.0, 300.0, 0.0, TargetClass::FIGHTER, 2.0, 0.8, radar::kSensorTargeting)};
    fusion.step(reports.data(), reports.size(), true, false, 1);
    EXPECT_EQ(1U, fusion.epochs()) << "the first frame runs one";
    EXPECT_TRUE(fusion.take_update());
    for (std::uint32_t frame = 2; frame <= 25; ++frame) {
        fusion.step(reports.data(), reports.size(), true, false, frame);
    }
    EXPECT_EQ(1U, fusion.epochs());
    EXPECT_FALSE(fusion.take_update());
    fusion.step(reports.data(), reports.size(), true, false, 26);
    EXPECT_EQ(2U, fusion.epochs());
    EXPECT_TRUE(fusion.take_update());
    fusion.step(nullptr, 0, false, false, 51);
    EXPECT_EQ(0U, fusion.count()) << "a sensor's dropped tracks are gone at once";
    EXPECT_EQ(0U, fusion.block().track_count);
    EXPECT_FALSE(fusion.block().targeting_present);
}

}  // namespace
