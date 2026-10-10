#ifndef FMOFP_BUS_COMMAND_WORD_HPP
#define FMOFP_BUS_COMMAND_WORD_HPP

#include <cstdint>

#include <fmofp/bus/status.hpp>
#include <fmofp/bus/word.hpp>

// MIL-STD-1553B command word (ICD README section 3):
//   bits 15-11  RT address, 31 = broadcast
//   bit  10     T/R: 1 = the terminal transmits, 0 = it receives
//   bits 9-5    subaddress; 0 or 31 means the low field is a mode code
//   bits 4-0    data word count (0 encodes 32), or the mode code number

namespace fmofp::bus {

constexpr std::uint8_t kBroadcastRt = 31U;
constexpr std::uint8_t kMaxRt = 31U;
constexpr std::uint8_t kMaxSubaddress = 31U;
constexpr std::uint8_t kMaxDataWords = 32U;
constexpr std::uint8_t kMaxModeCode = 31U;

struct CommandWord {
    std::uint8_t rt{0};
    bool transmit{false};
    std::uint8_t subaddress{0};
    // Data word count 1..32 for a data message; the mode code 0..31 when
    // subaddress is 0 or 31.
    std::uint8_t count_or_mode{0};

    [[nodiscard]] constexpr bool is_mode_code() const noexcept {
        return subaddress == 0U || subaddress == kMaxSubaddress;
    }
    [[nodiscard]] constexpr bool is_broadcast() const noexcept { return rt == kBroadcastRt; }
};

// kInvalidArgument when a field is out of range (RT or subaddress above 31, a
// data word count outside 1..32, a mode code above 31).
[[nodiscard]] Status encode_command_word(const CommandWord& cw, Word& out) noexcept;

// Every 16-bit pattern is a valid command word, so this cannot fail; a zero
// count field decodes as 32 data words.
CommandWord decode_command_word(Word in) noexcept;

}  // namespace fmofp::bus

#endif
