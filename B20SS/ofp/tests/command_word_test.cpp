#include <gtest/gtest.h>

#include <cstdint>

#include <fmofp/bus/command_word.hpp>

namespace {

using fmofp::Status;
using fmofp::bus::CommandWord;
using fmofp::bus::decode_command_word;
using fmofp::bus::encode_command_word;
using fmofp::bus::kBroadcastRt;
using fmofp::bus::Word;

TEST(CommandWord, EncodesTheIcdLayout) {
    // RT 9 (radar), transmit, SA 1 (weather radar), 2 data words: RDR_WX_STATUS.
    CommandWord cw;
    cw.rt = 9;
    cw.transmit = true;
    cw.subaddress = 1;
    cw.count_or_mode = 2;
    Word w = 0;
    ASSERT_EQ(Status::kOk, encode_command_word(cw, w));
    EXPECT_EQ(Word{0x4C22}, w);  // 01001 1 00001 00010
}

TEST(CommandWord, ThirtyTwoWordsIsSentAsZero) {
    CommandWord cw;
    cw.rt = 5;
    cw.transmit = false;
    cw.subaddress = 11;
    cw.count_or_mode = 32;
    Word w = 0;
    ASSERT_EQ(Status::kOk, encode_command_word(cw, w));
    EXPECT_EQ(Word{0}, w & Word{0x1F});
    const CommandWord back = decode_command_word(w);
    EXPECT_EQ(32, back.count_or_mode);
    EXPECT_FALSE(back.is_mode_code());
}

TEST(CommandWord, ModeCodeKeepsItsNumber) {
    // Broadcast SYNCHRONIZE_WITH_DATA: RT 31, receive, SA 31, mode code 17.
    CommandWord cw;
    cw.rt = kBroadcastRt;
    cw.transmit = false;
    cw.subaddress = 31;
    cw.count_or_mode = 17;
    Word w = 0;
    ASSERT_EQ(Status::kOk, encode_command_word(cw, w));
    EXPECT_EQ(Word{0xFBF1}, w);  // 11111 0 11111 10001
    const CommandWord back = decode_command_word(w);
    EXPECT_TRUE(back.is_mode_code());
    EXPECT_TRUE(back.is_broadcast());
    EXPECT_EQ(17, back.count_or_mode);

    // Mode code 0 on subaddress 0 stays 0, not 32.
    cw.subaddress = 0;
    cw.count_or_mode = 0;
    ASSERT_EQ(Status::kOk, encode_command_word(cw, w));
    EXPECT_EQ(0, decode_command_word(w).count_or_mode);
}

TEST(CommandWord, RejectsOutOfRangeFields) {
    const auto rejects = [](std::uint8_t rt, std::uint8_t sa, std::uint8_t count) {
        CommandWord cw;
        cw.rt = rt;
        cw.subaddress = sa;
        cw.count_or_mode = count;
        Word w = 0xFFFF;
        const Status s = encode_command_word(cw, w);
        EXPECT_EQ(Word{0}, w) << "output must be zeroed on failure";
        return s == Status::kInvalidArgument;
    };
    EXPECT_TRUE(rejects(32, 1, 1));   // RT above 31
    EXPECT_TRUE(rejects(1, 32, 1));   // subaddress above 31
    EXPECT_TRUE(rejects(1, 1, 0));    // a data message needs at least one word
    EXPECT_TRUE(rejects(1, 1, 33));   // and at most 32
    EXPECT_TRUE(rejects(1, 31, 32));  // mode codes go up to 31
}

TEST(CommandWord, EveryPatternRoundTrips) {
    for (std::uint32_t pattern = 0; pattern <= 0xFFFFU; ++pattern) {
        const auto w = static_cast<Word>(pattern);
        const CommandWord cw = decode_command_word(w);
        Word again = 0;
        ASSERT_EQ(Status::kOk, encode_command_word(cw, again)) << "pattern " << pattern;
        ASSERT_EQ(w, again) << "pattern " << pattern;
    }
}

}  // namespace
