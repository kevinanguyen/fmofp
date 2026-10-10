#ifndef FMOFP_BUS_STATUS_HPP
#define FMOFP_BUS_STATUS_HPP

#include <cstdint>

namespace fmofp {

// Result of any operation that can fail. There are no exceptions in flight code
// (CLAUDE.md), so every such function returns one of these and the caller handles
// it. The values are ordered: a larger value is a worse outcome, which lets the
// worst of several results be combined with worst().
enum class Status : std::uint8_t {
    kOk = 0,
    // Success, but a value was limited to the range its field can carry. The
    // producer should mark that field NO_COMPUTED_DATA (ICD README section 3).
    kClamped = 1,
    // A received enum field holds a value the ICD does not define.
    kInvalidEnum = 2,
    // The caller passed something unusable: NaN, an address out of range, ...
    kInvalidArgument = 3,
    // A received word has a reserved bit set or otherwise breaks the word format.
    kMalformed = 4,
    kBadLength = 5,
    kBadMagic = 6,
    kTimeout = 7,
    kNotReady = 8,
    kIoError = 9,
};

// True for the two outcomes that still produced usable output.
constexpr bool ok(Status s) noexcept { return s == Status::kOk || s == Status::kClamped; }

constexpr Status worst(Status a, Status b) noexcept {
    return static_cast<std::uint8_t>(a) >= static_cast<std::uint8_t>(b) ? a : b;
}

// For logs and test output only; never for control flow.
const char* to_string(Status s) noexcept;

}  // namespace fmofp

#endif
