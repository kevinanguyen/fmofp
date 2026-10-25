#include <fmofp/bus/block.hpp>

namespace fmofp::bus {

std::uint16_t crc16_ccitt_false(const Word* words, std::uint16_t count) noexcept {
    std::uint16_t crc = 0xFFFFU;
    if (words == nullptr) {
        return crc;
    }
    for (std::uint16_t i = 0; i < count; ++i) {
        const std::array<std::uint8_t, 2> bytes{static_cast<std::uint8_t>(words[i] >> 8U),
                                                static_cast<std::uint8_t>(words[i] & 0xFFU)};
        for (const std::uint8_t byte : bytes) {
            crc = static_cast<std::uint16_t>(crc ^ (static_cast<unsigned>(byte) << 8U));
            for (unsigned bit = 0; bit < 8U; ++bit) {
                const unsigned shifted = static_cast<unsigned>(crc) << 1U;
                crc = (crc & 0x8000U) != 0U ? static_cast<std::uint16_t>(shifted ^ 0x1021U)
                                            : static_cast<std::uint16_t>(shifted);
            }
        }
    }
    return crc;
}

void encode_descriptor(const BlockDescriptor& d, Word* out) noexcept {
    out[0] = static_cast<Word>(static_cast<unsigned>(d.version) << 8U);
    out[1] = d.length;
    out[2] = d.segments;
    out[3] = d.crc;
}

void decode_descriptor(const Word* in, BlockDescriptor& d) noexcept {
    d.version = static_cast<std::uint8_t>(in[0] >> 8U);
    d.length = in[1];
    d.segments = in[2];
    d.crc = in[3];
}

// ---- transmitter ---------------------------------------------------------

Status BlockTransmitter::init(Word* storage, std::uint16_t capacity) noexcept {
    if (storage == nullptr || capacity == 0U || capacity > kMaxBlockPayloadWords) {
        return Status::kInvalidArgument;
    }
    storage_ = storage;
    capacity_ = capacity;
    length_ = 0;
    segments_ = 0;
    crc_ = 0;
    next_ = 0;
    version_ = 0;
    return Status::kOk;
}

Status BlockTransmitter::set_payload(const Word* payload, std::uint16_t count) noexcept {
    if (storage_ == nullptr || (payload == nullptr && count != 0U)) {
        return Status::kInvalidArgument;
    }
    if (count > capacity_) {
        return Status::kBadLength;
    }
    for (std::uint16_t i = 0; i < count; ++i) {
        storage_[i] = payload[i];
    }
    length_ = count;
    segments_ = block_segment_count(count);
    crc_ = crc16_ccitt_false(storage_, count);
    next_ = 0;
    version_ = static_cast<std::uint8_t>(version_ == 255U ? 1U : version_ + 1U);
    return Status::kOk;
}

void BlockTransmitter::descriptor(Word* out) const noexcept {
    BlockDescriptor d;
    d.version = version_;
    d.length = length_;
    d.segments = segments_;
    d.crc = crc_;
    encode_descriptor(d, out);
}

void BlockTransmitter::segment(std::uint16_t index, Word* out) const noexcept {
    out[0] = static_cast<Word>((static_cast<unsigned>(version_) << 8U) | (index & 0xFFU));
    const std::uint32_t base = static_cast<std::uint32_t>(index) * kBlockSegmentPayloadWords;
    for (std::uint16_t i = 0; i < kBlockSegmentPayloadWords; ++i) {
        const std::uint32_t at = base + i;
        out[1U + i] = (version_ != 0U && at < length_) ? storage_[at] : Word{0};
    }
}

void BlockTransmitter::advance() noexcept {
    if (segments_ == 0U) {
        next_ = 0;
        return;
    }
    next_ = static_cast<std::uint16_t>(next_ + 1U);
    if (next_ >= segments_) {
        next_ = 0;
    }
}

// ---- receiver ------------------------------------------------------------

Status BlockReceiver::init(Word* storage, std::uint16_t capacity) noexcept {
    if (storage == nullptr || capacity == 0U || capacity > kMaxBlockPayloadWords) {
        return Status::kInvalidArgument;
    }
    storage_ = storage;
    capacity_ = capacity;
    completed_blocks_ = 0;
    restart(0);
    return Status::kOk;
}

void BlockReceiver::restart(std::uint8_t new_version) noexcept {
    collecting_ = new_version;
    have_descriptor_ = false;
    descriptor_ = BlockDescriptor{};
    received_ = {};
    complete_ = false;
}

bool BlockReceiver::feed_descriptor(const Word* in) noexcept {
    if (storage_ == nullptr || in == nullptr) {
        return false;
    }
    BlockDescriptor d;
    decode_descriptor(in, d);
    if (d.version == 0U) {
        return false;
    }
    if (d.version != collecting_) {
        restart(d.version);
    }
    descriptor_ = d;
    have_descriptor_ = true;
    return check_complete();
}

bool BlockReceiver::feed_segment(const Word* in) noexcept {
    if (storage_ == nullptr || in == nullptr) {
        return false;
    }
    const auto segment_version = static_cast<std::uint8_t>(in[0] >> 8U);
    const std::uint16_t index = in[0] & 0xFFU;
    if (segment_version == 0U) {
        return false;
    }
    if (segment_version != collecting_) {
        restart(segment_version);
    }
    const std::uint32_t base = static_cast<std::uint32_t>(index) * kBlockSegmentPayloadWords;
    for (std::uint16_t i = 0; i < kBlockSegmentPayloadWords; ++i) {
        const std::uint32_t at = base + i;
        if (at < capacity_) {
            storage_[at] = in[1U + i];
        }
    }
    received_[index / 32U] |= static_cast<std::uint32_t>(1U) << (index % 32U);
    return check_complete();
}

bool BlockReceiver::check_complete() noexcept {
    const bool was_complete = complete_;
    complete_ = false;
    if (!have_descriptor_ || descriptor_.length > capacity_ ||
        descriptor_.segments != block_segment_count(descriptor_.length) ||
        descriptor_.segments > kMaxBlockSegments) {
        return false;
    }
    for (std::uint16_t i = 0; i < descriptor_.segments; ++i) {
        if ((received_[i / 32U] & (static_cast<std::uint32_t>(1U) << (i % 32U))) == 0U) {
            return false;
        }
    }
    complete_ = crc16_ccitt_false(storage_, descriptor_.length) == descriptor_.crc;
    if (complete_ && !was_complete) {
        ++completed_blocks_;
        return true;
    }
    return false;
}

Status BlockPublisher::init(Word* storage, std::uint16_t capacity) noexcept {
    segment_reads_ = 0;
    published_ = false;
    return tx_.init(storage, capacity);
}

Status BlockPublisher::publish(const RemoteTerminal& terminal, std::uint8_t segment_sa, const Word* payload,
                               std::uint16_t count) noexcept {
    const Status s = tx_.set_payload(payload, count);
    if (s != Status::kOk) {
        return s;
    }
    segment_reads_ = terminal.transmit_count(segment_sa);
    published_ = true;
    return Status::kOk;
}

Status BlockPublisher::serve(RemoteTerminal& terminal, std::uint8_t descriptor_sa, std::uint8_t segment_sa) noexcept {
    const std::uint32_t reads = terminal.transmit_count(segment_sa);
    if (!published_ && reads != segment_reads_) {
        tx_.advance();
    }
    segment_reads_ = reads;
    published_ = false;
    std::array<Word, kBlockDescriptorWords> descriptor{};
    tx_.descriptor(descriptor.data());
    const Status d = terminal.set_transmit(descriptor_sa, descriptor.data(), kBlockDescriptorWords);
    std::array<Word, kBlockSegmentWords> segment{};
    tx_.next_segment(segment.data());
    return worst(d, terminal.set_transmit(segment_sa, segment.data(), kBlockSegmentWords));
}

Status BlockCollector::init(Word* storage, std::uint16_t capacity) noexcept { return rx_.init(storage, capacity); }

bool BlockCollector::collect(RemoteTerminal& terminal, std::uint8_t descriptor_sa, std::uint8_t segment_sa) noexcept {
    bool done = false;
    std::array<Word, kBlockDescriptorWords> descriptor{};
    std::uint8_t count = 0;
    bool fresh = false;
    if (terminal.get_received(descriptor_sa, descriptor.data(), kBlockDescriptorWords, count, fresh) == Status::kOk &&
        fresh && count == kBlockDescriptorWords) {
        done = rx_.feed_descriptor(descriptor.data());
    }
    std::array<Word, kBlockSegmentWords> segment{};
    count = 0;
    fresh = false;
    if (terminal.get_received(segment_sa, segment.data(), kBlockSegmentWords, count, fresh) == Status::kOk && fresh &&
        count == kBlockSegmentWords) {
        done = rx_.feed_segment(segment.data()) || done;
    }
    return done;
}

}  // namespace fmofp::bus
