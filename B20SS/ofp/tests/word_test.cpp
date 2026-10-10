#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include <fmofp/bus/word.hpp>

namespace {

using fmofp::Status;
using fmofp::bus::decode_scaled;
using fmofp::bus::encode_scaled;
using fmofp::bus::field_mask;
using fmofp::bus::from_twos;
using fmofp::bus::get_bit;
using fmofp::bus::get_field;
using fmofp::bus::set_bit;
using fmofp::bus::set_field;
using fmofp::bus::to_twos;
using fmofp::bus::Word;

// The ICD's angle scale, 180/32768 deg per count, reduced as Fraction does.
constexpr std::int64_t kAngleNum = 45;
constexpr std::int64_t kAngleDen = 8192;

TEST(FieldMask, CoversTheStatedBits) {
    EXPECT_EQ(Word{0xFFFF}, field_mask(15U, 0U));
    EXPECT_EQ(Word{0x000F}, field_mask(3U, 0U));
    EXPECT_EQ(Word{0xF000}, field_mask(15U, 12U));
    EXPECT_EQ(Word{0x0080}, field_mask(7U, 7U));
    EXPECT_EQ(Word{0x0C00}, field_mask(11U, 10U));
}

TEST(Field, SetThenGetReturnsTheValue) {
    Word w = 0;
    w = set_field(w, 15U, 12U, Word{0xA});
    w = set_field(w, 11U, 8U, Word{0x5});
    w = set_field(w, 7U, 0U, Word{0x3C});
    EXPECT_EQ(Word{0xA53C}, w);
    EXPECT_EQ(Word{0xA}, get_field(w, 15U, 12U));
    EXPECT_EQ(Word{0x5}, get_field(w, 11U, 8U));
    EXPECT_EQ(Word{0x3C}, get_field(w, 7U, 0U));
}

TEST(Field, SetLeavesOtherBitsAloneAndDropsExcessBits) {
    const Word w = set_field(Word{0xFFFF}, 7U, 4U, Word{0x0});
    EXPECT_EQ(Word{0xFF0F}, w);
    // 0x1F does not fit in 4 bits; only its low 4 bits land in the field.
    EXPECT_EQ(Word{0x00F0}, set_field(Word{0}, 7U, 4U, Word{0x1F}));
}

TEST(Bit, SetAndGet) {
    Word w = 0;
    w = set_bit(w, 15U, true);
    w = set_bit(w, 0U, true);
    EXPECT_EQ(Word{0x8001}, w);
    EXPECT_TRUE(get_bit(w, 15U));
    EXPECT_FALSE(get_bit(w, 14U));
    EXPECT_TRUE(get_bit(w, 0U));
    EXPECT_EQ(Word{0x0001}, set_bit(w, 15U, false));
}

TEST(TwosComplement, SixteenBits) {
    EXPECT_EQ(Word{0xFFFF}, to_twos(-1, 16U));
    EXPECT_EQ(Word{0x8000}, to_twos(-32768, 16U));
    EXPECT_EQ(Word{0x7FFF}, to_twos(32767, 16U));
    EXPECT_EQ(-1, from_twos(Word{0xFFFF}, 16U));
    EXPECT_EQ(-32768, from_twos(Word{0x8000}, 16U));
    EXPECT_EQ(32767, from_twos(Word{0x7FFF}, 16U));
    EXPECT_EQ(0, from_twos(Word{0}, 16U));
}

TEST(TwosComplement, NarrowFieldsSignExtend) {
    EXPECT_EQ(Word{0xF}, to_twos(-1, 4U));
    EXPECT_EQ(Word{0x8}, to_twos(-8, 4U));
    EXPECT_EQ(-1, from_twos(Word{0xF}, 4U));
    EXPECT_EQ(-8, from_twos(Word{0x8}, 4U));
    EXPECT_EQ(7, from_twos(Word{0x7}, 4U));
    // Bits above the field are ignored.
    EXPECT_EQ(7, from_twos(Word{0xFFF7}, 4U));
}

TEST(TwosComplement, RoundTripsEveryEightBitValue) {
    for (std::int32_t v = -128; v <= 127; ++v) {
        EXPECT_EQ(v, from_twos(to_twos(v, 8U), 8U));
    }
}

TEST(EncodeScaled, ExactAnglesFromTheIcd) {
    std::int32_t raw = 0;
    EXPECT_EQ(Status::kOk, encode_scaled(45.0, kAngleNum, kAngleDen, -32768, 32767, raw));
    EXPECT_EQ(8192, raw);
    EXPECT_EQ(Status::kOk, encode_scaled(-180.0, kAngleNum, kAngleDen, -32768, 32767, raw));
    EXPECT_EQ(-32768, raw);
    EXPECT_EQ(Status::kOk, encode_scaled(179.9945, kAngleNum, kAngleDen, -32768, 32767, raw));
    EXPECT_EQ(32767, raw);
    EXPECT_EQ(Status::kOk, encode_scaled(0.0, kAngleNum, kAngleDen, -32768, 32767, raw));
    EXPECT_EQ(0, raw);
}

TEST(EncodeScaled, RoundsToNearestCount) {
    std::int32_t raw = 0;
    EXPECT_EQ(Status::kOk, encode_scaled(1.4, 1, 1, 0, 100, raw));
    EXPECT_EQ(1, raw);
    EXPECT_EQ(Status::kOk, encode_scaled(1.6, 1, 1, 0, 100, raw));
    EXPECT_EQ(2, raw);
    // A hair below zero on an unsigned field still rounds to a legal 0.
    EXPECT_EQ(Status::kOk, encode_scaled(-0.3, 1, 1, 0, 100, raw));
    EXPECT_EQ(0, raw);
}

TEST(EncodeScaled, ClampsAndSaysSo) {
    std::int32_t raw = 0;
    EXPECT_EQ(Status::kClamped, encode_scaled(200.0, kAngleNum, kAngleDen, -32768, 32767, raw));
    EXPECT_EQ(32767, raw);
    EXPECT_EQ(Status::kClamped, encode_scaled(-200.0, kAngleNum, kAngleDen, -32768, 32767, raw));
    EXPECT_EQ(-32768, raw);
    EXPECT_EQ(Status::kClamped, encode_scaled(-0.6, 1, 1, 0, 100, raw));
    EXPECT_EQ(0, raw);
    EXPECT_EQ(Status::kClamped, encode_scaled(1.0e300, 1, 1, 0, 100, raw));
    EXPECT_EQ(100, raw);
    EXPECT_EQ(Status::kClamped, encode_scaled(-1.0e300, 1, 1, 0, 100, raw));
    EXPECT_EQ(0, raw);
}

TEST(EncodeScaled, ClampsWhenOnlyTheRoundingCrossesTheLimit) {
    // 100.6 is within a count of the limit, so it reaches the rounding step;
    // it rounds to 101, which is still out of range.
    std::int32_t raw = 0;
    EXPECT_EQ(Status::kClamped, encode_scaled(100.6, 1, 1, 0, 100, raw));
    EXPECT_EQ(100, raw);
    // 100.4 rounds to 100 and is fine.
    EXPECT_EQ(Status::kOk, encode_scaled(100.4, 1, 1, 0, 100, raw));
    EXPECT_EQ(100, raw);
}

TEST(EncodeScaled, RejectsUnusableInput) {
    std::int32_t raw = 7;
    EXPECT_EQ(Status::kInvalidArgument,
              encode_scaled(std::numeric_limits<double>::quiet_NaN(), 1, 1, 0, 100, raw));
    EXPECT_EQ(0, raw);
    EXPECT_EQ(Status::kInvalidArgument,
              encode_scaled(std::numeric_limits<double>::infinity(), 1, 1, 0, 100, raw));
    EXPECT_EQ(Status::kInvalidArgument, encode_scaled(1.0, 0, 1, 0, 100, raw));
    EXPECT_EQ(Status::kInvalidArgument, encode_scaled(1.0, 1, 0, 0, 100, raw));
    EXPECT_EQ(Status::kInvalidArgument, encode_scaled(1.0, 1, 1, 10, 0, raw));
}

TEST(DecodeScaled, IsExactForIcdScales) {
    EXPECT_DOUBLE_EQ(45.0, decode_scaled(8192, kAngleNum, kAngleDen));
    EXPECT_DOUBLE_EQ(-180.0, decode_scaled(-32768, kAngleNum, kAngleDen));
    EXPECT_DOUBLE_EQ(12.34, decode_scaled(1234, 1, 100));
    EXPECT_DOUBLE_EQ(-0.5, decode_scaled(-32, 1, 64));
}

TEST(Scaled, RoundTripsAcrossTheWholeRange) {
    for (std::int32_t raw = -32768; raw <= 32767; raw += 997) {
        std::int32_t back = 0;
        const double value = decode_scaled(raw, kAngleNum, kAngleDen);
        ASSERT_EQ(Status::kOk, encode_scaled(value, kAngleNum, kAngleDen, -32768, 32767, back));
        EXPECT_EQ(raw, back);
    }
}

}  // namespace
