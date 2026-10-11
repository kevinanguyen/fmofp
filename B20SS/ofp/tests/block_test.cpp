#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include <fmofp/bus/block.hpp>

namespace {

using fmofp::Status;
using fmofp::bus::Word;
namespace bus = fmofp::bus;

std::vector<Word> payload_of(std::uint16_t length) {
    std::vector<Word> p(length);
    for (std::uint16_t i = 0; i < length; ++i) {
        p[i] = static_cast<Word>(i * 3U + 1U);
    }
    return p;
}

TEST(BlockCrc, KnownValues) {
    // CRC-16/CCITT-FALSE: 0xFFFF for nothing, 0x1D0F for two zero bytes.
    EXPECT_EQ(0xFFFFU, bus::crc16_ccitt_false(nullptr, 0));
    const Word zero = 0;
    EXPECT_EQ(0x1D0FU, bus::crc16_ccitt_false(&zero, 1));
    // Changing any bit changes the CRC.
    const std::vector<Word> p = payload_of(40);
    std::vector<Word> q = p;
    q[17] ^= 0x0400U;
    EXPECT_NE(bus::crc16_ccitt_false(p.data(), 40), bus::crc16_ccitt_false(q.data(), 40));
}

TEST(BlockTransmitter, DescriptorAndSegments) {
    std::array<Word, 100> storage{};
    bus::BlockTransmitter tx;
    ASSERT_EQ(Status::kOk, tx.init(storage.data(), static_cast<std::uint16_t>(storage.size())));
    EXPECT_FALSE(tx.has_payload());
    std::array<Word, bus::kBlockDescriptorWords> d{};
    tx.descriptor(d.data());
    EXPECT_EQ(Word{0}, d[0]) << "version 0 means nothing published yet";

    const std::vector<Word> p = payload_of(70);
    ASSERT_EQ(Status::kOk, tx.set_payload(p.data(), 70));
    EXPECT_TRUE(tx.has_payload());
    EXPECT_EQ(1U, tx.version());
    EXPECT_EQ(70U, tx.length());
    EXPECT_EQ(3U, tx.segment_count());
    EXPECT_EQ(0U, tx.next_index());
    tx.descriptor(d.data());
    EXPECT_EQ(Word{0x0100}, d[0]);
    EXPECT_EQ(Word{70}, d[1]);
    EXPECT_EQ(Word{3}, d[2]);
    EXPECT_EQ(bus::crc16_ccitt_false(p.data(), 70), d[3]);

    std::array<Word, bus::kBlockSegmentWords> s{};
    tx.segment(0, s.data());
    EXPECT_EQ(Word{0x0100}, s[0]);
    for (std::size_t i = 0; i < 31; ++i) {
        EXPECT_EQ(p[i], s[1 + i]);
    }
    tx.segment(2, s.data());
    EXPECT_EQ(Word{0x0102}, s[0]);
    for (std::size_t i = 0; i < 31; ++i) {
        EXPECT_EQ(62 + i < 70 ? p[62 + i] : Word{0}, s[1 + i]) << "the last segment is zero-padded";
    }

    tx.advance();
    tx.advance();
    EXPECT_EQ(2U, tx.next_index());
    tx.advance();
    EXPECT_EQ(0U, tx.next_index()) << "wraps so a late consumer still gets every segment";

    ASSERT_EQ(Status::kOk, tx.set_payload(p.data(), 31));
    EXPECT_EQ(2U, tx.version());
    EXPECT_EQ(1U, tx.segment_count());
    EXPECT_EQ(Status::kBadLength, tx.set_payload(p.data(), 101U));
}

TEST(BlockTransmitter, VersionSkipsZero) {
    std::array<Word, 8> storage{};
    bus::BlockTransmitter tx;
    ASSERT_EQ(Status::kOk, tx.init(storage.data(), 8));
    const Word w = 7;
    for (int i = 0; i < 255; ++i) {
        ASSERT_EQ(Status::kOk, tx.set_payload(&w, 1));
    }
    EXPECT_EQ(255U, tx.version());
    ASSERT_EQ(Status::kOk, tx.set_payload(&w, 1));
    EXPECT_EQ(1U, tx.version());
}

class BlockRoundTrip : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_EQ(Status::kOk, tx.init(tx_storage.data(), static_cast<std::uint16_t>(tx_storage.size())));
        ASSERT_EQ(Status::kOk, rx.init(rx_storage.data(), static_cast<std::uint16_t>(rx_storage.size())));
    }

    bool deliver_descriptor() {
        std::array<Word, bus::kBlockDescriptorWords> d{};
        tx.descriptor(d.data());
        return rx.feed_descriptor(d.data());
    }

    bool deliver_segment(std::uint16_t index) {
        std::array<Word, bus::kBlockSegmentWords> s{};
        tx.segment(index, s.data());
        return rx.feed_segment(s.data());
    }

    std::array<Word, 200> tx_storage{};
    std::array<Word, 200> rx_storage{};
    bus::BlockTransmitter tx;
    bus::BlockReceiver rx;
};

TEST_F(BlockRoundTrip, CompletesWhenEverySegmentAndTheDescriptorHaveArrived) {
    const std::vector<Word> p = payload_of(100);  // 4 segments
    ASSERT_EQ(Status::kOk, tx.set_payload(p.data(), 100));
    EXPECT_FALSE(deliver_descriptor());
    EXPECT_FALSE(deliver_segment(0));
    EXPECT_FALSE(deliver_segment(2));
    EXPECT_FALSE(deliver_segment(1));
    EXPECT_FALSE(rx.complete());
    EXPECT_TRUE(deliver_segment(3)) << "order does not matter; the last missing one completes it";
    EXPECT_TRUE(rx.complete());
    EXPECT_EQ(100U, rx.length());
    EXPECT_EQ(1U, rx.version());
    EXPECT_EQ(1U, rx.completed_blocks());
    for (std::size_t i = 0; i < 100; ++i) {
        EXPECT_EQ(p[i], rx.payload()[i]);
    }
    EXPECT_FALSE(deliver_segment(1)) << "a repeated segment reports nothing new";
    EXPECT_TRUE(rx.complete());
}

TEST_F(BlockRoundTrip, ADescriptorArrivingLastCompletesIt) {
    const std::vector<Word> p = payload_of(40);
    ASSERT_EQ(Status::kOk, tx.set_payload(p.data(), 40));
    EXPECT_FALSE(deliver_segment(0));
    EXPECT_FALSE(deliver_segment(1));
    EXPECT_TRUE(deliver_descriptor());
}

TEST_F(BlockRoundTrip, ANewVersionStartsOver) {
    const std::vector<Word> p = payload_of(40);
    ASSERT_EQ(Status::kOk, tx.set_payload(p.data(), 40));
    EXPECT_FALSE(deliver_descriptor());
    EXPECT_FALSE(deliver_segment(0));
    // The producer publishes again before the consumer saw segment 1.
    std::vector<Word> q = payload_of(40);
    q[35] = 0xBEEF;
    ASSERT_EQ(Status::kOk, tx.set_payload(q.data(), 40));
    EXPECT_FALSE(deliver_segment(1)) << "segment 1 of version 2 does not finish version 1";
    EXPECT_FALSE(rx.complete());
    EXPECT_EQ(2U, rx.version());
    EXPECT_FALSE(deliver_descriptor());
    EXPECT_TRUE(deliver_segment(0));
    EXPECT_EQ(Word{0xBEEF}, rx.payload()[35]);
}

TEST_F(BlockRoundTrip, CorruptionAndZeroVersionAreRefused) {
    const std::vector<Word> p = payload_of(40);
    ASSERT_EQ(Status::kOk, tx.set_payload(p.data(), 40));
    EXPECT_FALSE(deliver_descriptor());
    EXPECT_FALSE(deliver_segment(0));
    std::array<Word, bus::kBlockSegmentWords> s{};
    tx.segment(1, s.data());
    s[5] ^= 0x0001U;
    EXPECT_FALSE(rx.feed_segment(s.data())) << "the CRC catches a flipped bit";
    EXPECT_FALSE(rx.complete());
    EXPECT_TRUE(deliver_segment(1)) << "the good copy repairs it";

    const std::array<Word, bus::kBlockSegmentWords> zero{};
    EXPECT_FALSE(rx.feed_segment(zero.data()));
    EXPECT_TRUE(rx.complete()) << "an unloaded (all-zero) subaddress disturbs nothing";
    const std::array<Word, bus::kBlockDescriptorWords> zero_descriptor{};
    EXPECT_FALSE(rx.feed_descriptor(zero_descriptor.data()));
    EXPECT_TRUE(rx.complete());
}

TEST_F(BlockRoundTrip, ABlockLargerThanTheReceiverNeverCompletes) {
    std::array<Word, 50> small{};
    bus::BlockReceiver tiny;
    ASSERT_EQ(Status::kOk, tiny.init(small.data(), 50));
    const std::vector<Word> p = payload_of(100);
    ASSERT_EQ(Status::kOk, tx.set_payload(p.data(), 100));
    std::array<Word, bus::kBlockDescriptorWords> d{};
    tx.descriptor(d.data());
    EXPECT_FALSE(tiny.feed_descriptor(d.data()));
    for (std::uint16_t i = 0; i < 4; ++i) {
        std::array<Word, bus::kBlockSegmentWords> s{};
        tx.segment(i, s.data());
        EXPECT_FALSE(tiny.feed_segment(s.data()));
    }
    EXPECT_FALSE(tiny.complete());
}

TEST(BlockInit, RejectsBadStorage) {
    std::array<Word, 8> storage{};
    bus::BlockTransmitter tx;
    EXPECT_EQ(Status::kInvalidArgument, tx.init(nullptr, 8));
    EXPECT_EQ(Status::kInvalidArgument, tx.init(storage.data(), 0));
    bus::BlockReceiver rx;
    EXPECT_EQ(Status::kInvalidArgument, rx.init(storage.data(), bus::kMaxBlockPayloadWords + 1U));
    EXPECT_EQ(Status::kInvalidArgument, tx.set_payload(storage.data(), 1)) << "not initialised";
}

}  // namespace
