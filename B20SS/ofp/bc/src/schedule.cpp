#include <fmofp/bc/schedule.hpp>

#include <fmofp/bus/command_word.hpp>

namespace fmofp::bc {

namespace {

constexpr unsigned kSyncMajorHi = 15U;
constexpr unsigned kSyncMajorLo = 6U;
constexpr unsigned kSyncMinorHi = 5U;
constexpr unsigned kSyncMinorLo = 0U;

bool valid_rate(std::uint8_t rate_hz) noexcept {
    return rate_hz == 50 || rate_hz == 10 || rate_hz == 5 || rate_hz == 1;
}

std::uint8_t period_frames(std::uint8_t rate_hz) noexcept {
    return static_cast<std::uint8_t>(kMinorFramesPerMajor / rate_hz);
}

std::uint32_t message_us(std::uint8_t words) noexcept {
    // Command word, the data, and the status word, plus the gaps around them.
    return (2U + words) * kUsPerWord + kUsPerMessageGap;
}

}  // namespace

Status Schedule::add(const ScheduledMessage& message) noexcept {
    if (!valid_rate(message.rate_hz) || message.words < 1 || message.words > bus::kMaxDataWords ||
        message.rt >= bus::kBroadcastRt || message.subaddress < 1 || message.subaddress > 30) {
        return Status::kInvalidArgument;
    }
    if (message.relay_rt != 0U &&
        (!message.transmit || message.relay_rt >= bus::kBroadcastRt || message.relay_rt == message.rt ||
         message.relay_subaddress < 1 || message.relay_subaddress > 30)) {
        return Status::kInvalidArgument;
    }
    if (count_ >= kMaxScheduledMessages) {
        return Status::kBadLength;
    }
    messages_[count_] = message;
    ++count_;
    built_ = false;
    return Status::kOk;
}

Status Schedule::build() noexcept {
    frames_ = {};
    // One rotating phase per period, so consecutive messages of the same rate
    // land in different minor frames.
    std::array<std::uint8_t, kMinorFramesPerMajor + 1> next_phase{};

    for (std::size_t i = 0; i < count_; ++i) {
        const ScheduledMessage& m = messages_[i];
        const std::uint8_t period = period_frames(m.rate_hz);
        const std::uint8_t phase = next_phase[period];
        next_phase[period] = static_cast<std::uint8_t>((phase + 1U) % period);

        for (std::uint8_t minor = phase; minor < kMinorFramesPerMajor;
             minor = static_cast<std::uint8_t>(minor + period)) {
            FrameSlots& f = frames_[minor];
            if (f.count >= kMaxSlotsPerFrame) {
                frames_ = {};
                return Status::kBadLength;
            }
            Slot& s = f.slots[f.count];
            s.rt = m.rt;
            s.subaddress = m.subaddress;
            s.transmit = m.transmit;
            s.words = m.words;
            s.relay_rt = m.relay_rt;
            s.relay_subaddress = m.relay_subaddress;
            ++f.count;
        }
    }
    built_ = true;
    return Status::kOk;
}

const FrameSlots& Schedule::frame(std::uint8_t minor) const noexcept {
    return frames_[minor < kMinorFramesPerMajor ? minor : 0U];
}

std::uint32_t Schedule::frame_load_us(std::uint8_t minor) const noexcept {
    const FrameSlots& f = frame(minor);
    std::uint32_t us = 2U * kUsPerWord + kUsPerMessageGap;  // the sync broadcast
    for (std::size_t i = 0; i < f.count; ++i) {
        us += message_us(f.slots[i].words);
        if (f.slots[i].has_relay()) {
            us += message_us(f.slots[i].words);  // the same words go out again to the consumer
        }
    }
    return us;
}

std::uint32_t Schedule::worst_frame_load_us() const noexcept {
    std::uint32_t max_us = 0;
    for (std::uint8_t minor = 0; minor < kMinorFramesPerMajor; ++minor) {
        const std::uint32_t us = frame_load_us(minor);
        max_us = us > max_us ? us : max_us;
    }
    return max_us;
}

bus::Word encode_sync_word(std::uint16_t major, std::uint8_t minor) noexcept {
    bus::Word w = 0;
    w = bus::set_field(w, kSyncMajorHi, kSyncMajorLo, static_cast<bus::Word>(major % kMajorFrameModulus));
    w = bus::set_field(w, kSyncMinorHi, kSyncMinorLo, minor);
    return w;
}

void decode_sync_word(bus::Word w, std::uint16_t& major, std::uint8_t& minor) noexcept {
    major = bus::get_field(w, kSyncMajorHi, kSyncMajorLo);
    minor = static_cast<std::uint8_t>(bus::get_field(w, kSyncMinorHi, kSyncMinorLo));
}

}  // namespace fmofp::bc
