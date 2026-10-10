#ifndef FMOFP_BUS_TRANSPORT_HPP
#define FMOFP_BUS_TRANSPORT_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/status.hpp>
#include <fmofp/bus/word.hpp>

// The datagram that carries one bus transaction on the simulated wire
// (ICD README section 7). Four big-endian header words, then the bus words:
//   0  magic 0x1553
//   1  bits 15-8 frame type, bits 7-0 flags (bit 0 = injected parity error)
//   2  transaction number (the BC counts; the RT echoes)
//   3  number of bus words that follow
// Only the byte layout lives here; what the words mean is the RT and BC's job.

namespace fmofp::bus {

constexpr Word kFrameMagic = 0x1553U;
constexpr std::size_t kFrameHeaderWords = 4;
// A monitor record of an RT-to-RT transfer is the largest: two command
// words, 32 data words and two status words.
constexpr std::size_t kMaxFrameWords = 36;
constexpr std::size_t kMaxFrameBytes = 2 * (kFrameHeaderWords + kMaxFrameWords);

enum class FrameType : std::uint8_t {
    kBcToRt = 1,
    kRtToBc = 2,
    kMonitor = 3,
};

constexpr std::uint8_t kFlagParityError = 0x01U;

struct Frame {
    FrameType type{FrameType::kBcToRt};
    std::uint8_t flags{0};
    Word transaction{0};
    std::uint8_t word_count{0};
    std::array<Word, kMaxFrameWords> words{};
};

// Writes the frame as bytes. kInvalidArgument if word_count exceeds
// kMaxFrameWords or the buffer is too small; `length` is the bytes written.
[[nodiscard]] Status encode_frame(const Frame& frame, std::uint8_t* out, std::size_t capacity,
                                  std::size_t& length) noexcept;

// Reads a frame from bytes. kBadMagic, kBadLength (odd length, too short,
// or word_count disagreeing with the bytes present) or kMalformed (unknown
// frame type). On failure `frame` is left untouched.
[[nodiscard]] Status decode_frame(const std::uint8_t* in, std::size_t length, Frame& frame) noexcept;

}  // namespace fmofp::bus

#endif
