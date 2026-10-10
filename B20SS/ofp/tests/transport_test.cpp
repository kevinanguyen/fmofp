#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/transport.hpp>

namespace {

using fmofp::Status;
using fmofp::bus::decode_frame;
using fmofp::bus::encode_frame;
using fmofp::bus::Frame;
using fmofp::bus::FrameType;
using fmofp::bus::kFlagParityError;
using fmofp::bus::kFrameHeaderWords;
using fmofp::bus::kMaxFrameBytes;
using fmofp::bus::kMaxFrameWords;
using fmofp::bus::Word;

Frame sample() {
    Frame f;
    f.type = FrameType::kBcToRt;
    f.flags = 0;
    f.transaction = 0x1234;
    f.word_count = 3;
    f.words[0] = 0x4C22;  // command word: RT 9, T, SA 1, 2 words
    f.words[1] = 0xBEEF;
    f.words[2] = 0x0001;
    return f;
}

TEST(Frame, BytesAreBigEndianInTheDocumentedOrder) {
    std::array<std::uint8_t, kMaxFrameBytes> bytes{};
    std::size_t len = 0;
    ASSERT_EQ(Status::kOk, encode_frame(sample(), bytes.data(), bytes.size(), len));
    ASSERT_EQ(2 * (kFrameHeaderWords + 3), len);
    const std::uint8_t expected[] = {0x15, 0x53, 0x01, 0x00, 0x12, 0x34, 0x00,
                                     0x03, 0x4C, 0x22, 0xBE, 0xEF, 0x00, 0x01};
    for (std::size_t i = 0; i < sizeof expected; ++i) {
        EXPECT_EQ(expected[i], bytes[i]) << "byte " << i;
    }
}

TEST(Frame, RoundTripsWithFlagsAndEveryType) {
    for (const FrameType type : {FrameType::kBcToRt, FrameType::kRtToBc, FrameType::kMonitor}) {
        Frame f = sample();
        f.type = type;
        f.flags = kFlagParityError;
        std::array<std::uint8_t, kMaxFrameBytes> bytes{};
        std::size_t len = 0;
        ASSERT_EQ(Status::kOk, encode_frame(f, bytes.data(), bytes.size(), len));
        Frame back;
        ASSERT_EQ(Status::kOk, decode_frame(bytes.data(), len, back));
        EXPECT_EQ(type, back.type);
        EXPECT_EQ(kFlagParityError, back.flags);
        EXPECT_EQ(Word{0x1234}, back.transaction);
        EXPECT_EQ(3, back.word_count);
        EXPECT_EQ(f.words, back.words);
    }
}

TEST(Frame, LargestFrameFitsTheBuffer) {
    Frame f = sample();
    f.type = FrameType::kMonitor;
    f.word_count = kMaxFrameWords;
    for (std::size_t i = 0; i < kMaxFrameWords; ++i) {
        f.words[i] = static_cast<Word>(i * 0x0101U);
    }
    std::array<std::uint8_t, kMaxFrameBytes> bytes{};
    std::size_t len = 0;
    ASSERT_EQ(Status::kOk, encode_frame(f, bytes.data(), bytes.size(), len));
    EXPECT_EQ(kMaxFrameBytes, len);
    Frame back;
    ASSERT_EQ(Status::kOk, decode_frame(bytes.data(), len, back));
    EXPECT_EQ(f.words, back.words);
}

TEST(Frame, EncodeRejectsWhatCannotBeSent) {
    std::array<std::uint8_t, kMaxFrameBytes> bytes{};
    std::size_t len = 7;
    Frame f = sample();
    f.word_count = kMaxFrameWords + 1;
    EXPECT_EQ(Status::kInvalidArgument, encode_frame(f, bytes.data(), bytes.size(), len));
    EXPECT_EQ(0U, len);
    f = sample();
    EXPECT_EQ(Status::kInvalidArgument, encode_frame(f, bytes.data(), 13, len));  // one byte short
    f.type = static_cast<FrameType>(9);
    EXPECT_EQ(Status::kInvalidArgument, encode_frame(f, bytes.data(), bytes.size(), len));
}

TEST(Frame, DecodeRejectsDamagedBytesAndLeavesTheOutputAlone) {
    std::array<std::uint8_t, kMaxFrameBytes> bytes{};
    std::size_t len = 0;
    ASSERT_EQ(Status::kOk, encode_frame(sample(), bytes.data(), bytes.size(), len));

    Frame untouched;
    untouched.transaction = 0x7777;
    const auto still_untouched = [&untouched] { return untouched.transaction == 0x7777; };

    EXPECT_EQ(Status::kBadLength, decode_frame(bytes.data(), len - 1, untouched));  // odd
    EXPECT_EQ(Status::kBadLength, decode_frame(bytes.data(), len - 2, untouched));  // count disagrees
    EXPECT_EQ(Status::kBadLength, decode_frame(bytes.data(), 6, untouched));        // no full header
    EXPECT_EQ(Status::kBadLength, decode_frame(nullptr, len, untouched));

    std::array<std::uint8_t, kMaxFrameBytes> bad = bytes;
    bad[0] = 0x00;
    EXPECT_EQ(Status::kBadMagic, decode_frame(bad.data(), len, untouched));

    bad = bytes;
    bad[2] = 0x07;  // frame type 7 does not exist
    EXPECT_EQ(Status::kMalformed, decode_frame(bad.data(), len, untouched));

    bad = bytes;
    bad[7] = static_cast<std::uint8_t>(kMaxFrameWords + 1);  // claims more words than allowed
    EXPECT_EQ(Status::kBadLength, decode_frame(bad.data(), len, untouched));

    EXPECT_TRUE(still_untouched());
}

}  // namespace
