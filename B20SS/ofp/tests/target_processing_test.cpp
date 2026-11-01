// target_processor.py's signature analyser and stealth detector, and the fusion's threat rule, pinned.
#include <cmath>

#include <fmofp/radar/target_processing.hpp>
#include <gtest/gtest.h>

namespace {

namespace radar = fmofp::radar;
namespace icd = fmofp::icd;
using icd::IffIdentity;
using icd::TargetClass;
using icd::Threat;

TEST(TargetProcessing, TheNearestReferenceRcsNamesTheType) {
    // No scintillation (factor 1), slow and low: the RCS alone decides.
    EXPECT_EQ(TargetClass::FIGHTER, radar::analyse_signature(2.0, 1.0, 100.0, 0.0).type);
    EXPECT_EQ(TargetClass::BOMBER, radar::analyse_signature(40.0, 1.0, 100.0, 0.0).type);
    EXPECT_EQ(TargetClass::HIGH_ALT, radar::analyse_signature(10.0, 1.0, 100.0, 0.0).type);
    EXPECT_EQ(TargetClass::MISSILE, radar::analyse_signature(0.05, 1.0, 100.0, 0.0).type);
    EXPECT_EQ(TargetClass::HELICOPTER, radar::analyse_signature(3.0, 1.0, 100.0, 0.0).type);
    EXPECT_EQ(TargetClass::DRONE, radar::analyse_signature(0.01, 1.0, 100.0, 0.0).type);
    EXPECT_EQ(TargetClass::UNKNOWN, radar::analyse_signature(5.0, 1.0, 100.0, 0.0).type);
    // A perfect match is 0.95 confident, as is anything within a factor of 1.4; the floor is 0.2.
    EXPECT_DOUBLE_EQ(0.95, radar::analyse_signature(2.0, 1.0, 100.0, 0.0).confidence);
    const radar::SignatureResult halved = radar::analyse_signature(2.0, 0.5, 100.0, 0.0);
    EXPECT_EQ(TargetClass::FIGHTER, halved.type) << "1 m2 is nearest the fighter's 2 in log space";
    EXPECT_NEAR(1.0 - std::log10(2.0) / 3.0, halved.confidence, 1e-12);
    const radar::SignatureResult nearer = radar::analyse_signature(2.0, 1.4, 100.0, 0.0);
    EXPECT_EQ(TargetClass::HELICOPTER, nearer.type) << "2.8 m2 is nearer the helicopter's 3";
    EXPECT_DOUBLE_EQ(0.95, nearer.confidence);
    EXPECT_NEAR(0.2, radar::analyse_signature(1.0e6, 1.0, 100.0, 0.0).confidence, 1e-12) << "the floor";
}

TEST(TargetProcessing, TheScintillationDrawMultipliesTheRcsWithAFloor) {
    EXPECT_DOUBLE_EQ(4.0, radar::analyse_signature(2.0, 2.0, 100.0, 0.0).rcs_corrected);
    EXPECT_DOUBLE_EQ(0.001, radar::analyse_signature(2.0, 0.0, 100.0, 0.0).rcs_corrected) << "max(0.001, ...)";
    EXPECT_EQ(TargetClass::DRONE, radar::analyse_signature(2.0, 0.0, 100.0, 0.0).type) << "0.001 m2 is nearest the drone";
}

TEST(TargetProcessing, SpeedAndAltitudeOverrideTheRcsMatch) {
    EXPECT_EQ(TargetClass::MISSILE, radar::analyse_signature(40.0, 1.0, 501.0, 0.0).type);
    EXPECT_EQ(TargetClass::FIGHTER, radar::analyse_signature(40.0, 1.0, 251.0, 5001.0).type);
    EXPECT_EQ(TargetClass::BOMBER, radar::analyse_signature(40.0, 1.0, 251.0, 5000.0).type) << "the altitude must exceed 5000";
    EXPECT_EQ(TargetClass::HIGH_ALT, radar::analyse_signature(40.0, 1.0, 100.0, 15001.0).type);
    EXPECT_EQ(TargetClass::MISSILE, radar::analyse_signature(40.0, 1.0, 600.0, 20000.0).type) << "speed first";
}

TEST(TargetProcessing, TheStealthDetectorUpdatesItsPosteriorPulseByPulse) {
    radar::StealthState state{};
    radar::StealthResult r = radar::assess_stealth(state, 0.05, 20.0);
    EXPECT_EQ(8U, r.pulses) << "max(8, 0 + 1)";
    EXPECT_NEAR(20.0 + 10.0 * std::log10(std::sqrt(8.0)), r.snr_integrated_db, 1e-12);
    EXPECT_NEAR(0.05 * 8.0 / (0.05 * 8.0 + 0.95), r.posterior, 1e-12);
    EXPECT_FALSE(r.is_stealth) << "0.296 is not yet above 0.6";
    r = radar::assess_stealth(state, 0.05, 20.0);
    EXPECT_EQ(9U, r.pulses) << "then one more pulse each detection";
    const double p1 = 0.05 * 8.0 / (0.05 * 8.0 + 0.95);
    EXPECT_NEAR(p1 * 8.0 / (p1 * 8.0 + 1.0 - p1), r.posterior, 1e-12);
    EXPECT_TRUE(r.is_stealth) << "0.77 above 0.6 and the RCS under 0.1";
    // A big RCS pulls the posterior down (ratio 0.2) and the flag off; the posterior never leaves 0.01..0.99.
    r = radar::assess_stealth(state, 5.0, 20.0);
    EXPECT_FALSE(r.is_stealth);
    EXPECT_LT(r.posterior, 0.77);
    for (int i = 0; i < 40; ++i) {
        r = radar::assess_stealth(state, 5.0, 20.0);
    }
    EXPECT_DOUBLE_EQ(0.01, r.posterior);
    radar::StealthState other{};
    for (int i = 0; i < 40; ++i) {
        r = radar::assess_stealth(other, 0.02, 20.0);
    }
    EXPECT_DOUBLE_EQ(0.99, r.posterior);
    // The middle band: 2 to 1 odds.
    radar::StealthState middle{};
    r = radar::assess_stealth(middle, 0.3, 20.0);
    EXPECT_NEAR(0.05 * 2.0 / (0.05 * 2.0 + 0.95), r.posterior, 1e-12);
}

TEST(TargetProcessing, TheThreatRule) {
    EXPECT_EQ(Threat::HOSTILE, radar::assess_threat(TargetClass::UNKNOWN, IffIdentity::HOSTILE, 50.0, 10.0));
    EXPECT_EQ(Threat::FRIENDLY, radar::assess_threat(TargetClass::FIGHTER, IffIdentity::FRIENDLY, 1.0, 400.0));
    EXPECT_EQ(Threat::HOSTILE, radar::assess_threat(TargetClass::FIGHTER, IffIdentity::UNKNOWN, 2.9, 201.0))
        << "small and fast";
    EXPECT_EQ(Threat::UNKNOWN, radar::assess_threat(TargetClass::FIGHTER, IffIdentity::UNKNOWN, 3.0, 201.0));
    EXPECT_EQ(Threat::UNKNOWN, radar::assess_threat(TargetClass::FIGHTER, IffIdentity::UNKNOWN, 2.9, 200.0));
    EXPECT_EQ(Threat::UNKNOWN, radar::assess_threat(TargetClass::STEALTH, IffIdentity::NEUTRAL, 2.9, 100.0));
}

}  // namespace
