#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include <fmofp/bus/status.hpp>

namespace {

using fmofp::ok;
using fmofp::Status;
using fmofp::to_string;
using fmofp::worst;

constexpr Status kAll[] = {
    Status::kOk,        Status::kClamped, Status::kInvalidEnum, Status::kInvalidArgument,
    Status::kMalformed, Status::kBadLength, Status::kBadMagic,  Status::kTimeout,
    Status::kNotReady,  Status::kIoError,
};

TEST(Status, OnlyOkAndClampedCountAsUsable) {
    for (const Status s : kAll) {
        EXPECT_EQ(s == Status::kOk || s == Status::kClamped, ok(s)) << to_string(s);
    }
}

TEST(Status, WorstPicksTheLargerCodeEitherWayRound) {
    EXPECT_EQ(Status::kClamped, worst(Status::kOk, Status::kClamped));
    EXPECT_EQ(Status::kClamped, worst(Status::kClamped, Status::kOk));
    EXPECT_EQ(Status::kInvalidArgument, worst(Status::kClamped, Status::kInvalidArgument));
    EXPECT_EQ(Status::kIoError, worst(Status::kIoError, Status::kTimeout));
    EXPECT_EQ(Status::kOk, worst(Status::kOk, Status::kOk));
}

TEST(Status, EveryValueHasItsOwnName) {
    for (const Status a : kAll) {
        ASSERT_NE(nullptr, to_string(a));
        EXPECT_STRNE("unknown status", to_string(a));
        for (const Status b : kAll) {
            if (a != b) {
                EXPECT_STRNE(to_string(a), to_string(b));
            }
        }
    }
    EXPECT_STREQ("unknown status", to_string(static_cast<Status>(0xFF)));
}

}  // namespace
