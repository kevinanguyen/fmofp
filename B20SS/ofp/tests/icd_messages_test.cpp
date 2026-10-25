// Pins the generated code to bit patterns worked out by hand from the ICD
// tables. The generated round-trip tests prove encode and decode agree with
// each other; these prove they agree with the document.
#include <gtest/gtest.h>

#include <cstdint>

#include <fmofp/icd/messages.hpp>
#include <fmofp/icd/version.hpp>

namespace {

using fmofp::Status;
using fmofp::bus::Word;
using namespace fmofp::icd;  // NOLINT: test

TEST(IcdVersion, MatchesTheDocument) {
    EXPECT_EQ(1, kIcdMajor);
    EXPECT_EQ(8, kIcdMinor);
}

TEST(FcsAttitude, ConstantsMatchTheIcd) {
    EXPECT_EQ(5, FcsAttitude::kRt);
    EXPECT_EQ(12, FcsAttitude::kSubaddress);
    EXPECT_EQ(Direction::kTransmit, FcsAttitude::kDirection);
    EXPECT_EQ(4, FcsAttitude::kWords);
    EXPECT_EQ(50, FcsAttitude::kRateHz);
}

TEST(FcsAttitude, AnglesLandWhereTheTableSays) {
    FcsAttitude m{};
    m.roll_ssm = Ssm::NORMAL;
    m.pitch_ssm = Ssm::NORMAL;
    m.yaw_ssm = Ssm::NORMAL;
    m.roll = 45.0;
    m.pitch = -90.0;
    m.yaw = 0.0;
    FcsAttitude::Words w{};
    ASSERT_EQ(Status::kOk, m.encode(w));
    EXPECT_EQ(Word{0xFC00}, w[0]);  // three NORMAL (3) SSMs in bits 15-14, 13-12, 11-10
    EXPECT_EQ(Word{0x2000}, w[1]);  // 45 deg = 8192 counts of 180/32768
    EXPECT_EQ(Word{0xC000}, w[2]);  // -90 deg = -16384
    EXPECT_EQ(Word{0x0000}, w[3]);
}

TEST(FcsAttitude, OutOfRangeIsClampedAndReported) {
    FcsAttitude m{};
    m.roll = 200.0;
    FcsAttitude::Words w{};
    EXPECT_EQ(Status::kClamped, m.encode(w));
    EXPECT_EQ(Word{0x7FFF}, w[1]);
    m.roll = -200.0;
    EXPECT_EQ(Status::kClamped, m.encode(w));
    EXPECT_EQ(Word{0x8000}, w[1]);
}

TEST(FcsAttitude, DefaultIsFailSafe) {
    // A default-constructed message encodes as all zeros, which decodes as
    // FAILURE_WARNING on every value: nothing is ever mistaken for good data.
    const FcsAttitude m{};
    FcsAttitude::Words w{};
    ASSERT_EQ(Status::kOk, m.encode(w));
    EXPECT_EQ((FcsAttitude::Words{}), w);
    EXPECT_EQ(Ssm::FAILURE_WARNING, m.roll_ssm);
}

TEST(TermStatus, LivesOnEveryTerminalAtSa29) {
    EXPECT_EQ(kEveryTerminal, TermStatus::kRt);
    EXPECT_EQ(29, TermStatus::kSubaddress);
    EXPECT_EQ(1, TermStatus::kRateHz);
    TermStatus m{};
    m.state = TerminalState::INITIALIZING;  // 15, the value -1 used to be
    m.last_cmd_seq = 0x42;
    m.healthy = true;
    m.icd_major = kIcdMajor;
    m.icd_minor = kIcdMinor;
    TermStatus::Words w{};
    ASSERT_EQ(Status::kOk, m.encode(w));
    EXPECT_EQ(Word{0xF042}, w[0]);
    EXPECT_EQ(Word{0x0001}, w[1]);
    EXPECT_EQ(static_cast<Word>((kIcdMajor << 8U) | kIcdMinor), w[3]);
}

TEST(RdrWxStatus, UndefinedModeIsRejected) {
    // weather_mode defines 0..5; 9 is not a mode.
    RdrWxStatus::Words w{};
    w[0] = static_cast<Word>((1U << 12U) | (9U << 8U));
    RdrWxStatus out{};
    out.last_cmd_seq = 77;
    EXPECT_EQ(Status::kInvalidEnum, RdrWxStatus::decode(w, out));
    EXPECT_EQ(77, out.last_cmd_seq) << "a failed decode must leave the output untouched";

    w[0] = static_cast<Word>((1U << 12U) | (1U << 8U));
    ASSERT_EQ(Status::kOk, RdrWxStatus::decode(w, out));
    EXPECT_EQ(TerminalState::NORMAL, out.state);
    EXPECT_EQ(WeatherMode::SURVEILLANCE, out.mode);
}

TEST(RdrWxStatus, EncodingAnUndefinedEnumIsAnError) {
    RdrWxStatus m{};
    m.mode = static_cast<WeatherMode>(9);
    RdrWxStatus::Words w{};
    EXPECT_EQ(Status::kInvalidArgument, m.encode(w));
}

TEST(FcsSurfaceCmd, PercentOfTravelInHundredths) {
    FcsSurfaceCmd m{};
    m.surface = ControlSurface::ELEVATOR;
    m.cmd_seq = 3;
    m.position = -12.34;
    FcsSurfaceCmd::Words w{};
    ASSERT_EQ(Status::kOk, m.encode(w));
    EXPECT_EQ(Word{0x0203}, w[0]);  // surface 2 in bits 11-8, cmd_seq in 7-0
    EXPECT_EQ(fmofp::bus::to_twos(-1234, 16U), w[1]);
    FcsSurfaceCmd d{};
    ASSERT_EQ(Status::kOk, FcsSurfaceCmd::decode(w, d));
    EXPECT_DOUBLE_EQ(-12.34, d.position);
}

TEST(FmsNavUpdateCmd, PresentBitsAndUnsignedScales) {
    FmsNavUpdateCmd m{};
    m.altitude_present = true;
    m.heading_present = true;
    m.cmd_seq = 0xAB;
    m.altitude = 35000.0;
    m.heading = 270.0;
    FmsNavUpdateCmd::Words w{};
    ASSERT_EQ(Status::kOk, m.encode(w));
    EXPECT_EQ(Word{0xA0AB}, w[0]);  // bits 15 and 13 set, cmd_seq low byte
    EXPECT_EQ(Word{35000}, w[1]);
    EXPECT_EQ(Word{0xC000}, w[3]);  // 270 deg of 360/65536 = 49152
}

TEST(LayoutOf, MfdCommandSharesTheRadarLayoutButNotItsAddress) {
    EXPECT_EQ(15, DspRdrWxCmd::kRt);
    EXPECT_EQ(20, DspRdrWxCmd::kSubaddress);
    EXPECT_EQ(Direction::kTransmit, DspRdrWxCmd::kDirection);
    EXPECT_EQ(10, DspRdrWxCmd::kRateHz);
    EXPECT_EQ(RdrWxModeCmd::kWords, DspRdrWxCmd::kWords);
    // The radar's own command is receive, aperiodic, at RT 9: nothing leaks across.
    EXPECT_EQ(9, RdrWxModeCmd::kRt);
    EXPECT_EQ(Direction::kReceive, RdrWxModeCmd::kDirection);

    DspRdrWxCmd m{};
    m.payload.requested_state = TerminalState::NORMAL;
    m.payload.requested_mode = WeatherMode::PRECIPITATION;
    m.payload.cmd_seq = 9;
    m.payload.range_km = 80.0;
    m.payload.tilt = 2.5;
    DspRdrWxCmd::Words w{};
    ASSERT_EQ(Status::kOk, m.encode(w));
    // The bus controller relays the words untouched into the radar's command.
    RdrWxModeCmd as_radar{};
    ASSERT_EQ(Status::kOk, RdrWxModeCmd::decode(w, as_radar));
    EXPECT_EQ(WeatherMode::PRECIPITATION, as_radar.requested_mode);
    EXPECT_EQ(9, as_radar.cmd_seq);
    EXPECT_DOUBLE_EQ(80.0, as_radar.range_km);
    EXPECT_NEAR(2.5, as_radar.tilt, 180.0 / 32768.0);
    DspRdrWxCmd back{};
    ASSERT_EQ(Status::kOk, DspRdrWxCmd::decode(w, back));
    EXPECT_EQ(9, back.payload.cmd_seq);
}

TEST(RdrWxModeCmd, ARangeBelowTheMinimumIsClampedToIt) {
    // range_km is 10..200 km, so a default-constructed command (0 km) cannot be
    // sent as-is: encode limits it and reports kClamped.
    const RdrWxModeCmd m{};
    RdrWxModeCmd::Words w{};
    EXPECT_EQ(Status::kClamped, m.encode(w));
    EXPECT_EQ(Word{10}, w[1]);
}

TEST(MessageTable, NoTwoMessagesShareAnAddress) {
    for (std::size_t i = 0; i < kMessages.size(); ++i) {
        for (std::size_t j = i + 1; j < kMessages.size(); ++j) {
            const MessageInfo& a = kMessages[i];
            const MessageInfo& b = kMessages[j];
            const bool same_rt = a.rt == b.rt || a.rt == kEveryTerminal || b.rt == kEveryTerminal;
            const bool clash = same_rt && a.subaddress == b.subaddress && a.direction == b.direction;
            EXPECT_FALSE(clash) << a.id << " and " << b.id;
        }
    }
    EXPECT_EQ(87U, kMessages.size());
    EXPECT_EQ(28U, kRelays.size());
}

}  // namespace
