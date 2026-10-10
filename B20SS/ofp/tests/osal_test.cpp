#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/osal/clock.hpp>
#include <fmofp/osal/udp.hpp>

namespace {

using fmofp::Status;
using fmofp::osal::kMaxDatagramBytes;
using fmofp::osal::kNsPerMs;
using fmofp::osal::Nanoseconds;
using fmofp::osal::now;
using fmofp::osal::sleep_for;
using fmofp::osal::sleep_until;
using fmofp::osal::UdpSocket;

// Ports well above the bus's 51000 block so a running simulator cannot clash.
constexpr std::uint16_t kPortA = 58101;
constexpr std::uint16_t kPortB = 58102;

// Generous: CI machines stall, and a late wake-up is not a bug of this code.
constexpr Nanoseconds kLateTolerance = 100 * kNsPerMs;

TEST(Clock, NowNeverGoesBackwards) {
    Nanoseconds previous = now();
    for (int i = 0; i < 10000; ++i) {
        const Nanoseconds t = now();
        ASSERT_GE(t, previous);
        previous = t;
    }
}

TEST(Clock, SleepUntilNeverWakesEarly) {
    for (const Nanoseconds ahead : {1 * kNsPerMs, 5 * kNsPerMs, 20 * kNsPerMs}) {
        const Nanoseconds deadline = now() + ahead;
        ASSERT_EQ(Status::kOk, sleep_until(deadline));
        const Nanoseconds woke = now();
        EXPECT_GE(woke, deadline);
        EXPECT_LT(woke, deadline + kLateTolerance);
    }
}

TEST(Clock, SleepUntilAPastDeadlineReturnsAtOnce) {
    const Nanoseconds start = now();
    EXPECT_EQ(Status::kOk, sleep_until(start - kNsPerMs));
    EXPECT_LT(now() - start, kLateTolerance);
    EXPECT_EQ(Status::kOk, sleep_for(0));
    EXPECT_EQ(Status::kOk, sleep_for(-5));
}

TEST(Clock, AbsoluteDeadlinesDoNotDrift) {
    // Five 10 ms frames scheduled from the previous deadline, not from the
    // wake-up time: the last deadline is exactly 50 ms after the first.
    const Nanoseconds first = now() + 10 * kNsPerMs;
    Nanoseconds deadline = first;
    for (int frame = 0; frame < 5; ++frame) {
        ASSERT_EQ(Status::kOk, sleep_until(deadline));
        deadline += 10 * kNsPerMs;
    }
    EXPECT_EQ(first + 50 * kNsPerMs, deadline);
    EXPECT_GE(now(), first + 40 * kNsPerMs);
}

class Udp : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_EQ(Status::kOk, fmofp::osal::init());
        ASSERT_EQ(Status::kOk, a.open(kPortA));
        ASSERT_EQ(Status::kOk, b.open(kPortB));
    }
    UdpSocket a;
    UdpSocket b;
};

TEST_F(Udp, DatagramArrivesIntactWithTheSendersPort) {
    std::array<std::uint8_t, 76> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = static_cast<std::uint8_t>(i * 3U);
    }
    ASSERT_EQ(Status::kOk, a.send_to(kPortB, out.data(), out.size()));

    std::array<std::uint8_t, kMaxDatagramBytes> in{};
    std::size_t len = 0;
    std::uint16_t from = 0;
    ASSERT_EQ(Status::kOk, b.receive(in.data(), in.size(), len, 500 * kNsPerMs, &from));
    ASSERT_EQ(out.size(), len);
    EXPECT_EQ(kPortA, from);
    for (std::size_t i = 0; i < out.size(); ++i) {
        EXPECT_EQ(out[i], in[i]) << "byte " << i;
    }
}

TEST_F(Udp, ReceiveTimesOutWhenNothingIsSent) {
    std::array<std::uint8_t, kMaxDatagramBytes> in{};
    std::size_t len = 99;
    const Nanoseconds start = now();
    EXPECT_EQ(Status::kTimeout, b.receive(in.data(), in.size(), len, 20 * kNsPerMs, nullptr));
    EXPECT_EQ(0U, len);
    EXPECT_GE(now() - start, 15 * kNsPerMs);
}

TEST_F(Udp, OversizedDatagramIsDroppedAndReported) {
    std::array<std::uint8_t, 64> out{};
    ASSERT_EQ(Status::kOk, a.send_to(kPortB, out.data(), out.size()));
    std::array<std::uint8_t, 32> small{};
    std::size_t len = 0;
    EXPECT_EQ(Status::kBadLength, b.receive(small.data(), small.size(), len, 500 * kNsPerMs, nullptr));
    // The next receive sees nothing: the oversized datagram is gone.
    EXPECT_EQ(Status::kTimeout, b.receive(small.data(), small.size(), len, 20 * kNsPerMs, nullptr));
}

TEST_F(Udp, RejectsBadArguments) {
    std::array<std::uint8_t, kMaxDatagramBytes + 1> too_big{};
    EXPECT_EQ(Status::kInvalidArgument, a.send_to(kPortB, too_big.data(), too_big.size()));
    EXPECT_EQ(Status::kInvalidArgument, a.send_to(kPortB, too_big.data(), 0));
    EXPECT_EQ(Status::kInvalidArgument, a.send_to(kPortB, nullptr, 4));
    std::size_t len = 0;
    EXPECT_EQ(Status::kInvalidArgument, a.receive(nullptr, 16, len, 0, nullptr));
    UdpSocket closed;
    EXPECT_FALSE(closed.is_open());
    EXPECT_EQ(Status::kInvalidArgument, closed.send_to(kPortB, too_big.data(), 4));
}

TEST_F(Udp, APortCannotBeOpenedTwice) {
    UdpSocket c;
    EXPECT_EQ(Status::kIoError, c.open(kPortA));
    EXPECT_FALSE(c.is_open());
}

}  // namespace
