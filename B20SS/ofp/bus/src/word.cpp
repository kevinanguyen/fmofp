#include <fmofp/bus/word.hpp>

#include <cmath>

namespace fmofp::bus {

Status encode_scaled(double value, std::int64_t lsb_num, std::int64_t lsb_den, std::int32_t raw_min,
                     std::int32_t raw_max, std::int32_t& raw) noexcept {
    raw = 0;
    if (!std::isfinite(value) || lsb_num <= 0 || lsb_den <= 0 || raw_min > raw_max) {
        return Status::kInvalidArgument;
    }

    const double counts = value * static_cast<double>(lsb_den) / static_cast<double>(lsb_num);
    const double lo = static_cast<double>(raw_min);
    const double hi = static_cast<double>(raw_max);

    // Anything more than a whole count outside the range is out of range however
    // it rounds. Deciding that here also keeps llround away from values it
    // cannot represent.
    if (counts < lo - 1.0) {
        raw = raw_min;
        return Status::kClamped;
    }
    if (counts > hi + 1.0) {
        raw = raw_max;
        return Status::kClamped;
    }

    const long long rounded = std::llround(counts);
    if (rounded < raw_min) {
        raw = raw_min;
        return Status::kClamped;
    }
    if (rounded > raw_max) {
        raw = raw_max;
        return Status::kClamped;
    }
    raw = static_cast<std::int32_t>(rounded);
    return Status::kOk;
}

double decode_scaled(std::int32_t raw, std::int64_t lsb_num, std::int64_t lsb_den) noexcept {
    // Multiply first: for every LSB in the ICD, raw * lsb_num is an exact
    // integer in double, so the single division is the only rounding step.
    return static_cast<double>(raw) * static_cast<double>(lsb_num) / static_cast<double>(lsb_den);
}

}  // namespace fmofp::bus
