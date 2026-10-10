#include <fmofp/bus/transport.hpp>

namespace fmofp::bus {

namespace {

void put_word(std::uint8_t* out, std::size_t index, Word w) noexcept {
    out[2 * index] = static_cast<std::uint8_t>(w >> 8U);
    out[2 * index + 1] = static_cast<std::uint8_t>(w & 0xFFU);
}

Word get_word(const std::uint8_t* in, std::size_t index) noexcept {
    return static_cast<Word>((static_cast<unsigned>(in[2 * index]) << 8U) | in[2 * index + 1]);
}

bool valid_type(std::uint8_t raw) noexcept {
    return raw == static_cast<std::uint8_t>(FrameType::kBcToRt) ||
           raw == static_cast<std::uint8_t>(FrameType::kRtToBc) ||
           raw == static_cast<std::uint8_t>(FrameType::kMonitor);
}

}  // namespace

Status encode_frame(const Frame& frame, std::uint8_t* out, std::size_t capacity,
                    std::size_t& length) noexcept {
    length = 0;
    if (out == nullptr || frame.word_count > kMaxFrameWords ||
        !valid_type(static_cast<std::uint8_t>(frame.type))) {
        return Status::kInvalidArgument;
    }
    const std::size_t needed = 2 * (kFrameHeaderWords + frame.word_count);
    if (capacity < needed) {
        return Status::kInvalidArgument;
    }
    put_word(out, 0, kFrameMagic);
    put_word(out, 1, static_cast<Word>((static_cast<unsigned>(frame.type) << 8U) | frame.flags));
    put_word(out, 2, frame.transaction);
    put_word(out, 3, frame.word_count);
    for (std::size_t i = 0; i < frame.word_count; ++i) {
        put_word(out, kFrameHeaderWords + i, frame.words[i]);
    }
    length = needed;
    return Status::kOk;
}

Status decode_frame(const std::uint8_t* in, std::size_t length, Frame& frame) noexcept {
    if (in == nullptr || length % 2 != 0 || length < 2 * kFrameHeaderWords) {
        return Status::kBadLength;
    }
    if (get_word(in, 0) != kFrameMagic) {
        return Status::kBadMagic;
    }
    const Word type_and_flags = get_word(in, 1);
    const auto type_raw = static_cast<std::uint8_t>(type_and_flags >> 8U);
    if (!valid_type(type_raw)) {
        return Status::kMalformed;
    }
    const Word count = get_word(in, 3);
    if (count > kMaxFrameWords || length != 2 * (kFrameHeaderWords + count)) {
        return Status::kBadLength;
    }

    Frame f{};
    f.type = static_cast<FrameType>(type_raw);
    f.flags = static_cast<std::uint8_t>(type_and_flags & 0xFFU);
    f.transaction = get_word(in, 2);
    f.word_count = static_cast<std::uint8_t>(count);
    for (std::size_t i = 0; i < count; ++i) {
        f.words[i] = get_word(in, kFrameHeaderWords + i);
    }
    frame = f;
    return Status::kOk;
}

}  // namespace fmofp::bus
