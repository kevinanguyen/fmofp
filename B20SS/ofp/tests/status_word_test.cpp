#include <gtest/gtest.h>

#include <cstdint>

#include <fmofp/bus/status_word.hpp>

namespace {

using fmofp::Status;
using fmofp::bus::decode_status_word;
using fmofp::bus::encode_status_word;
using fmofp::bus::StatusWord;
using fmofp::bus::Word;

TEST(StatusWord, EncodesTheIcdLayout) {
    StatusWord sw;
    sw.rt = 9;
    sw.message_error = true;
    sw.terminal_flag = true;
    Word w = 0;
    ASSERT_EQ(Status::kOk, encode_status_word(sw, w));
    EXPECT_EQ(Word{0x4C01}, w);  // 01001 1 0 0 000 0 0 0 0 1

    sw = StatusWord{};
    sw.rt = 31;
    sw.service_request = true;
    sw.broadcast_received = true;
    sw.busy = true;
    sw.subsystem_flag = true;
    ASSERT_EQ(Status::kOk, encode_status_word(sw, w));
    EXPECT_EQ(Word{0xF91C}, w);  // 11111 0 0 1 000 1 1 1 0 0
}

TEST(StatusWord, RejectsBadRt) {
    StatusWord sw;
    sw.rt = 32;
    Word w = 0xFFFF;
    EXPECT_EQ(Status::kInvalidArgument, encode_status_word(sw, w));
    EXPECT_EQ(Word{0}, w);
}

TEST(StatusWord, ReservedBitsAreMalformedButStillDecoded) {
    StatusWord sw;
    // RT 9 with the instrumentation bit (9) set.
    EXPECT_EQ(Status::kMalformed, decode_status_word(Word{0x4A00}, sw));
    EXPECT_EQ(9, sw.rt);
    // Reserved bit 5.
    EXPECT_EQ(Status::kMalformed, decode_status_word(Word{0x0020}, sw));
    // Dynamic bus control acceptance, bit 1.
    EXPECT_EQ(Status::kMalformed, decode_status_word(Word{0x0002}, sw));
    // Nothing reserved: fine.
    EXPECT_EQ(Status::kOk, decode_status_word(Word{0x4C1D}, sw));
    EXPECT_TRUE(sw.message_error);
    EXPECT_TRUE(sw.broadcast_received);
    EXPECT_TRUE(sw.busy);
    EXPECT_TRUE(sw.subsystem_flag);
    EXPECT_TRUE(sw.terminal_flag);
    EXPECT_FALSE(sw.service_request);
}

TEST(StatusWord, EveryLegalCombinationRoundTrips) {
    for (std::uint8_t rt = 0; rt <= 31; ++rt) {
        for (unsigned flags = 0; flags < 64U; ++flags) {
            StatusWord sw;
            sw.rt = rt;
            sw.message_error = (flags & 1U) != 0U;
            sw.service_request = (flags & 2U) != 0U;
            sw.broadcast_received = (flags & 4U) != 0U;
            sw.busy = (flags & 8U) != 0U;
            sw.subsystem_flag = (flags & 16U) != 0U;
            sw.terminal_flag = (flags & 32U) != 0U;
            Word w = 0;
            ASSERT_EQ(Status::kOk, encode_status_word(sw, w));
            StatusWord back;
            ASSERT_EQ(Status::kOk, decode_status_word(w, back));
            EXPECT_EQ(sw.rt, back.rt);
            EXPECT_EQ(sw.message_error, back.message_error);
            EXPECT_EQ(sw.service_request, back.service_request);
            EXPECT_EQ(sw.broadcast_received, back.broadcast_received);
            EXPECT_EQ(sw.busy, back.busy);
            EXPECT_EQ(sw.subsystem_flag, back.subsystem_flag);
            EXPECT_EQ(sw.terminal_flag, back.terminal_flag);
        }
    }
}

}  // namespace
