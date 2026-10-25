#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bc/icd_schedule.hpp>
#include <fmofp/bc/schedule.hpp>
#include <fmofp/icd/messages.hpp>

namespace {

using fmofp::Status;
using fmofp::bc::decode_sync_word;
using fmofp::bc::encode_sync_word;
using fmofp::bc::kMaxSlotsPerFrame;
using fmofp::bc::kMinorFrameBudgetUs;
using fmofp::bc::kMinorFramesPerMajor;
using fmofp::bc::Schedule;
using fmofp::bc::ScheduledMessage;
using fmofp::bus::Word;

ScheduledMessage msg(std::uint8_t rt, std::uint8_t sa, std::uint8_t rate, std::uint8_t words = 2,
                     bool transmit = true) {
    ScheduledMessage m;
    m.rt = rt;
    m.subaddress = sa;
    m.transmit = transmit;
    m.words = words;
    m.rate_hz = rate;
    return m;
}

std::size_t slots_for(const Schedule& s, std::uint8_t rt, std::uint8_t sa) {
    std::size_t n = 0;
    for (std::uint8_t minor = 0; minor < kMinorFramesPerMajor; ++minor) {
        const auto& f = s.frame(minor);
        for (std::size_t i = 0; i < f.count; ++i) {
            if (f.slots[i].rt == rt && f.slots[i].subaddress == sa) {
                ++n;
            }
        }
    }
    return n;
}

TEST(Schedule, EachRateLandsTheRightNumberOfTimesPerSecond) {
    Schedule s;
    ASSERT_EQ(Status::kOk, s.add(msg(5, 12, 50)));
    ASSERT_EQ(Status::kOk, s.add(msg(5, 11, 10)));
    ASSERT_EQ(Status::kOk, s.add(msg(5, 10, 5)));
    ASSERT_EQ(Status::kOk, s.add(msg(5, 29, 1)));
    ASSERT_EQ(Status::kOk, s.build());
    EXPECT_TRUE(s.built());
    EXPECT_EQ(50U, slots_for(s, 5, 12));
    EXPECT_EQ(10U, slots_for(s, 5, 11));
    EXPECT_EQ(5U, slots_for(s, 5, 10));
    EXPECT_EQ(1U, slots_for(s, 5, 29));
}

TEST(Schedule, MessagesOfOneRateAreSpreadAcrossPhases) {
    Schedule s;
    for (std::uint8_t rt = 1; rt <= 10; ++rt) {
        ASSERT_EQ(Status::kOk, s.add(msg(rt, 29, 1)));
    }
    ASSERT_EQ(Status::kOk, s.build());
    // Ten 1 Hz messages: no minor frame carries more than one of them.
    for (std::uint8_t minor = 0; minor < kMinorFramesPerMajor; ++minor) {
        EXPECT_LE(s.frame(minor).count, 1U) << "minor " << static_cast<int>(minor);
    }
    // And 5 Hz messages take different frames within their 10-frame period.
    Schedule t;
    ASSERT_EQ(Status::kOk, t.add(msg(1, 10, 5)));
    ASSERT_EQ(Status::kOk, t.add(msg(2, 10, 5)));
    ASSERT_EQ(Status::kOk, t.build());
    EXPECT_EQ(1U, t.frame(0).count);
    EXPECT_EQ(1U, t.frame(1).count);
    EXPECT_EQ(0U, t.frame(2).count);
    EXPECT_EQ(1U, t.frame(10).count);
}

TEST(Schedule, EachMessageTakesTheLeastLoadedPhaseOfItsPeriod) {
    // Two blocks' descriptor and segment slots at 25 Hz (v1.6): the two 32-word
    // segments must not share a frame, or the frame would carry both copies'
    // relays as well.
    Schedule s;
    ASSERT_EQ(Status::kOk, s.add(msg(9, 6, 25, 1)));
    ASSERT_EQ(Status::kOk, s.add(msg(9, 7, 25, 32)));
    ASSERT_EQ(Status::kOk, s.add(msg(9, 14, 25, 1)));
    ASSERT_EQ(Status::kOk, s.add(msg(9, 15, 25, 32)));
    ASSERT_EQ(Status::kOk, s.build());
    EXPECT_EQ(25U, slots_for(s, 9, 7));
    for (std::uint8_t minor = 0; minor < kMinorFramesPerMajor; ++minor) {
        unsigned segments = 0;
        const auto& f = s.frame(minor);
        for (std::size_t i = 0; i < f.count; ++i) {
            segments += f.slots[i].words == 32U ? 1U : 0U;
        }
        EXPECT_EQ(1U, segments) << "minor " << static_cast<int>(minor);
    }
    EXPECT_EQ(s.frame_load_us(1) + 2U * (3U * 20U + 12U), s.frame_load_us(0)) << "both descriptors sit beside the first segment";

    // A 1 Hz poll goes to a frame the 10 Hz traffic left quiet, not to frame 0 by habit.
    Schedule t;
    ASSERT_EQ(Status::kOk, t.add(msg(27, 1, 10, 32)));  // frames 0, 5, 10, ...
    ASSERT_EQ(Status::kOk, t.add(msg(5, 29, 1, 2)));
    ASSERT_EQ(Status::kOk, t.build());
    std::uint8_t where = kMinorFramesPerMajor;
    for (std::uint8_t minor = 0; minor < kMinorFramesPerMajor; ++minor) {
        const auto& f = t.frame(minor);
        for (std::size_t i = 0; i < f.count; ++i) {
            if (f.slots[i].rt == 5U && f.slots[i].subaddress == 29U) {
                where = minor;
            }
        }
    }
    ASSERT_LT(where, kMinorFramesPerMajor);
    EXPECT_NE(0U, where % 5U) << "the 1 Hz poll shares no frame with the 10 Hz block segment";
}

TEST(Schedule, RejectsWhatTheBusCannotCarry) {
    Schedule s;
    EXPECT_EQ(Status::kInvalidArgument, s.add(msg(5, 12, 20)));       // not a rate group
    EXPECT_EQ(Status::kInvalidArgument, s.add(msg(5, 12, 40)));       // nor is 40 Hz
    EXPECT_EQ(Status::kInvalidArgument, s.add(msg(5, 12, 50, 0)));    // no words
    EXPECT_EQ(Status::kInvalidArgument, s.add(msg(5, 12, 50, 33)));   // too many
    EXPECT_EQ(Status::kInvalidArgument, s.add(msg(31, 12, 50)));      // broadcast is not a terminal
    EXPECT_EQ(Status::kInvalidArgument, s.add(msg(5, 0, 50)));        // mode code subaddress
    EXPECT_EQ(Status::kInvalidArgument, s.add(msg(5, 31, 50)));
    EXPECT_EQ(0U, s.message_count());

    // 33 messages at 50 Hz cannot fit a frame of 32 slots.
    for (std::uint8_t i = 0; i < 33; ++i) {
        ASSERT_EQ(Status::kOk, s.add(msg(static_cast<std::uint8_t>(1 + i % 30), 5, 50)));
    }
    EXPECT_EQ(Status::kBadLength, s.build());
    EXPECT_FALSE(s.built());
}

TEST(Schedule, LoadEstimateCountsEveryWordAndGap) {
    Schedule s;
    ASSERT_EQ(Status::kOk, s.build());  // empty: only the sync
    EXPECT_EQ(52U, s.frame_load_us(0));
    ASSERT_EQ(Status::kOk, s.add(msg(5, 12, 50, 4)));
    ASSERT_EQ(Status::kOk, s.build());
    EXPECT_EQ(52U + (2U + 4U) * 20U + 12U, s.frame_load_us(7));
    EXPECT_EQ(s.frame_load_us(7), s.worst_frame_load_us());
}

TEST(Schedule, RelaysRideWithTheirSourceAndCountTwice) {
    Schedule s;
    ScheduledMessage src = msg(27, 1, 50, 12);
    src.relays[0] = {5, 13};
    src.relay_count = 1;
    ASSERT_EQ(Status::kOk, s.add(src));
    ASSERT_EQ(Status::kOk, s.build());
    const auto& slot = s.frame(0).slots[0];
    EXPECT_TRUE(slot.has_relay());
    ASSERT_EQ(1U, slot.relay_count);
    EXPECT_EQ(5, slot.relays[0].rt);
    EXPECT_EQ(13, slot.relays[0].subaddress);
    EXPECT_EQ(52U + 2U * ((2U + 12U) * 20U + 12U), s.frame_load_us(0));

    // Three consumers: the words go out four times.
    Schedule three;
    ScheduledMessage fanout = src;
    fanout.relays[1] = {12, 13};
    fanout.relays[2] = {7, 13};
    fanout.relay_count = 3;
    ASSERT_EQ(Status::kOk, three.add(fanout));
    ASSERT_EQ(Status::kOk, three.build());
    EXPECT_EQ(3U, three.frame(0).slots[0].relay_count);
    EXPECT_EQ(52U + 4U * ((2U + 12U) * 20U + 12U), three.frame_load_us(0));

    // A relay needs a transmit source and a real, different consumer.
    ScheduledMessage bad = msg(27, 1, 50, 12, false);
    bad.relays[0] = {5, 13};
    bad.relay_count = 1;
    EXPECT_EQ(Status::kInvalidArgument, s.add(bad));
    bad = src;
    bad.relays[0].rt = 27;
    EXPECT_EQ(Status::kInvalidArgument, s.add(bad));
    bad = src;
    bad.relays[0].subaddress = 31;
    EXPECT_EQ(Status::kInvalidArgument, s.add(bad));
    bad = src;
    bad.relay_count = static_cast<std::uint8_t>(fmofp::bc::kMaxRelaysPerMessage + 1U);
    EXPECT_EQ(Status::kInvalidArgument, s.add(bad));
}

TEST(IcdSchedule, RelayTargetsArePresentOnlyWhenTheirConsumerIs) {
    // Plant and FCS both present: PLANT_SENSORS is polled and relayed to FCS SA 13,
    // and FCS_SENSORS gets no receive slot of its own.
    const std::array<std::uint8_t, 2> both{27, 5};
    Schedule s;
    ASSERT_EQ(Status::kOk, fmofp::bc::add_icd_messages(s, both.data(), both.size()));
    ASSERT_EQ(Status::kOk, s.build());
    std::size_t relayed = 0;
    for (std::uint8_t minor = 0; minor < kMinorFramesPerMajor; ++minor) {
        const auto& f = s.frame(minor);
        for (std::size_t i = 0; i < f.count; ++i) {
            EXPECT_FALSE(f.slots[i].rt == 5 && f.slots[i].subaddress == 13) << "FCS_SENSORS must not be polled";
            if (f.slots[i].rt == 27 && f.slots[i].subaddress == 1) {
                ASSERT_EQ(1U, f.slots[i].relay_count);
                EXPECT_EQ(5, f.slots[i].relays[0].rt);
                EXPECT_EQ(13, f.slots[i].relays[0].subaddress);
                ++relayed;
            }
        }
    }
    EXPECT_EQ(50U, relayed);

    // Plant alone: polled, not relayed.
    const std::array<std::uint8_t, 1> plant_only{27};
    Schedule p;
    ASSERT_EQ(Status::kOk, fmofp::bc::add_icd_messages(p, plant_only.data(), plant_only.size()));
    ASSERT_EQ(Status::kOk, p.build());
    EXPECT_FALSE(p.frame(0).slots[0].has_relay() || p.frame(0).slots[1].has_relay());

    // Plant, FCS, FMS and navigation: PLANT_SENSORS goes to all three consumers
    // (ICD relays order), and no consumer's copy is polled on its own.
    const std::array<std::uint8_t, 4> all{27, 5, 12, 7};
    Schedule a;
    ASSERT_EQ(Status::kOk, fmofp::bc::add_icd_messages(a, all.data(), all.size()));
    ASSERT_EQ(Status::kOk, a.build());
    bool seen = false;
    for (std::uint8_t minor = 0; minor < kMinorFramesPerMajor; ++minor) {
        const auto& f = a.frame(minor);
        for (std::size_t i = 0; i < f.count; ++i) {
            const fmofp::bc::Slot& slot = f.slots[i];
            EXPECT_FALSE(!slot.transmit && slot.subaddress == 13 && (slot.rt == 5 || slot.rt == 12 || slot.rt == 7))
                << "a relayed sensor copy must not be polled";
            if (slot.rt == 27 && slot.subaddress == 1) {
                ASSERT_EQ(3U, slot.relay_count);
                EXPECT_EQ(5, slot.relays[0].rt);
                EXPECT_EQ(12, slot.relays[1].rt);
                EXPECT_EQ(7, slot.relays[2].rt);
                EXPECT_EQ(13, slot.relays[0].subaddress);
                EXPECT_EQ(13, slot.relays[1].subaddress);
                EXPECT_EQ(13, slot.relays[2].subaddress);
                seen = true;
            }
        }
    }
    EXPECT_TRUE(seen);
    EXPECT_LE(a.worst_frame_load_us(), kMinorFrameBudgetUs);
}

TEST(SyncWord, PacksMajorAndMinor) {
    EXPECT_EQ(Word{0x0000}, encode_sync_word(0, 0));
    EXPECT_EQ(Word{0x0071}, encode_sync_word(1, 49));
    EXPECT_EQ(Word{0xFFC0}, encode_sync_word(1023, 0));
    EXPECT_EQ(Word{0x0000}, encode_sync_word(1024, 0)) << "the major count wraps at 1024";
    std::uint16_t major = 0;
    std::uint8_t minor = 0;
    decode_sync_word(Word{0x0C07}, major, minor);
    EXPECT_EQ(48, major);
    EXPECT_EQ(7, minor);
}

TEST(IcdSchedule, EveryLiveTerminalIsPolledAndTheBusStaysUnderBudget) {
    std::array<std::uint8_t, 23> all{};
    for (std::uint8_t i = 0; i < all.size(); ++i) {
        all[i] = static_cast<std::uint8_t>(i + 1);
    }
    Schedule s;
    ASSERT_EQ(Status::kOk, fmofp::bc::add_icd_messages(s, all.data(), all.size()));
    ASSERT_EQ(Status::kOk, s.build());
    EXPECT_GT(s.message_count(), all.size());
    for (const std::uint8_t rt : all) {
        EXPECT_EQ(1U, slots_for(s, rt, fmofp::icd::TermStatus::kSubaddress)) << "RT " << static_cast<int>(rt);
    }
    EXPECT_EQ(50U, slots_for(s, fmofp::icd::FcsAttitude::kRt, fmofp::icd::FcsAttitude::kSubaddress));
    EXPECT_LT(s.worst_frame_load_us(), kMinorFrameBudgetUs);

    // Only the listed terminals are scheduled.
    Schedule two;
    const std::array<std::uint8_t, 2> pair{1, 2};
    ASSERT_EQ(Status::kOk, fmofp::bc::add_icd_messages(two, pair.data(), pair.size()));
    ASSERT_EQ(Status::kOk, two.build());
    EXPECT_EQ(2U, two.message_count());
    EXPECT_EQ(0U, slots_for(two, fmofp::icd::FcsAttitude::kRt, fmofp::icd::FcsAttitude::kSubaddress));
}

}  // namespace
