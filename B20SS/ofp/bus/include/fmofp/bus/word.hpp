#ifndef FMOFP_BUS_WORD_HPP
#define FMOFP_BUS_WORD_HPP

#include <cstdint>

#include <fmofp/bus/status.hpp>

// The 16-bit bus word and the only sanctioned ways of putting fields into it.
// Bits are numbered as in the ICD: 15 is the most significant, 0 the least
// (ICD README section 3). Generated message code uses nothing but these.

namespace fmofp::bus {

using Word = std::uint16_t;

constexpr unsigned kWordBits = 16U;

// Mask covering bits hi..lo inclusive. Requires 15 >= hi >= lo.
constexpr Word field_mask(unsigned hi, unsigned lo) noexcept {
    const unsigned width = hi - lo + 1U;
    const std::uint32_t ones = width >= kWordBits ? 0xFFFFU : (1U << width) - 1U;
    return static_cast<Word>((ones << lo) & 0xFFFFU);
}

// The value of bits hi..lo, right-aligned.
constexpr Word get_field(Word w, unsigned hi, unsigned lo) noexcept {
    return static_cast<Word>((static_cast<std::uint32_t>(w) & field_mask(hi, lo)) >> lo);
}

// `w` with bits hi..lo replaced by the low bits of `value`; excess bits of
// `value` are discarded.
constexpr Word set_field(Word w, unsigned hi, unsigned lo, Word value) noexcept {
    const std::uint32_t mask = field_mask(hi, lo);
    const std::uint32_t cleared = static_cast<std::uint32_t>(w) & ~mask;
    const std::uint32_t placed = (static_cast<std::uint32_t>(value) << lo) & mask;
    return static_cast<Word>(cleared | placed);
}

constexpr bool get_bit(Word w, unsigned bit) noexcept { return get_field(w, bit, bit) != 0U; }

constexpr Word set_bit(Word w, unsigned bit, bool value) noexcept {
    return set_field(w, bit, bit, value ? Word{1} : Word{0});
}

// Two's-complement representation of `value` in a field `width` bits wide
// (1..16). The caller keeps `value` within the field's range; extra high bits
// are discarded.
constexpr Word to_twos(std::int32_t value, unsigned width) noexcept {
    return static_cast<Word>(static_cast<std::uint32_t>(value) & field_mask(width - 1U, 0U));
}

// Sign-extends the low `width` bits of `raw`.
constexpr std::int32_t from_twos(Word raw, unsigned width) noexcept {
    const std::uint32_t mask = field_mask(width - 1U, 0U);
    const std::uint32_t bits = static_cast<std::uint32_t>(raw) & mask;
    const std::uint32_t sign = 1U << (width - 1U);
    return (bits & sign) != 0U ? static_cast<std::int32_t>(bits) - static_cast<std::int32_t>(sign << 1U)
                               : static_cast<std::int32_t>(bits);
}

// The same for a 32-bit value carried in a double word (ICD README section
// 3): most significant word first.
constexpr std::uint32_t to_twos32(std::int32_t value) noexcept { return static_cast<std::uint32_t>(value); }

constexpr std::int32_t from_twos32(std::uint32_t raw) noexcept {
    constexpr std::uint32_t kSign = 0x80000000U;
    return (raw & kSign) != 0U ? static_cast<std::int32_t>(raw & 0x7FFFFFFFU) - 0x7FFFFFFF - 1
                               : static_cast<std::int32_t>(raw);
}

// Scaled quantities: engineering value = raw * lsb_num / lsb_den.
//
// encode_scaled rounds `value` to the nearest raw count and limits it to
// [raw_min, raw_max]. It returns kOk, kClamped when the value had to be
// limited, or kInvalidArgument for NaN, infinity or a bad scale (raw is then 0).
[[nodiscard]] Status encode_scaled(double value, std::int64_t lsb_num, std::int64_t lsb_den,
                                   std::int32_t raw_min, std::int32_t raw_max,
                                   std::int32_t& raw) noexcept;

double decode_scaled(std::int32_t raw, std::int64_t lsb_num, std::int64_t lsb_den) noexcept;

}  // namespace fmofp::bus

#endif
