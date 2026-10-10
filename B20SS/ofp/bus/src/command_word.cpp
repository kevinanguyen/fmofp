#include <fmofp/bus/command_word.hpp>

namespace fmofp::bus {

namespace {

constexpr unsigned kRtHi = 15U;
constexpr unsigned kRtLo = 11U;
constexpr unsigned kTrBit = 10U;
constexpr unsigned kSaHi = 9U;
constexpr unsigned kSaLo = 5U;
constexpr unsigned kCountHi = 4U;
constexpr unsigned kCountLo = 0U;

}  // namespace

Status encode_command_word(const CommandWord& cw, Word& out) noexcept {
    out = 0;
    if (cw.rt > kMaxRt || cw.subaddress > kMaxSubaddress) {
        return Status::kInvalidArgument;
    }
    if (cw.is_mode_code()) {
        if (cw.count_or_mode > kMaxModeCode) {
            return Status::kInvalidArgument;
        }
    } else if (cw.count_or_mode < 1U || cw.count_or_mode > kMaxDataWords) {
        return Status::kInvalidArgument;
    }

    // A count of 32 is sent as 0: the field is five bits wide.
    const Word count_field = cw.count_or_mode == kMaxDataWords ? Word{0} : Word{cw.count_or_mode};

    Word w = 0;
    w = set_field(w, kRtHi, kRtLo, cw.rt);
    w = set_bit(w, kTrBit, cw.transmit);
    w = set_field(w, kSaHi, kSaLo, cw.subaddress);
    w = set_field(w, kCountHi, kCountLo, count_field);
    out = w;
    return Status::kOk;
}

CommandWord decode_command_word(Word in) noexcept {
    CommandWord cw;
    cw.rt = static_cast<std::uint8_t>(get_field(in, kRtHi, kRtLo));
    cw.transmit = get_bit(in, kTrBit);
    cw.subaddress = static_cast<std::uint8_t>(get_field(in, kSaHi, kSaLo));
    const auto low = static_cast<std::uint8_t>(get_field(in, kCountHi, kCountLo));
    cw.count_or_mode = (!cw.is_mode_code() && low == 0U) ? kMaxDataWords : low;
    return cw;
}

}  // namespace fmofp::bus
