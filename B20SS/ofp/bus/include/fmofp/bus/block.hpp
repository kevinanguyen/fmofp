#ifndef FMOFP_BUS_BLOCK_HPP
#define FMOFP_BUS_BLOCK_HPP

#include <array>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/bus/word.hpp>

// Block transfers (ICD README section 8): a payload longer than one message
// travels as a 4-word descriptor and 32-word segments on two transmit
// subaddresses. BlockTransmitter is the producer's side, BlockReceiver the
// consumer's. Neither owns memory: the partition hands each a buffer it
// allocated statically. BlockPublisher and BlockCollector put them on a
// RemoteTerminal's subaddresses, which is all a partition process has to do.

namespace fmofp::bus {

constexpr std::uint8_t kBlockDescriptorWords = 4U;
constexpr std::uint8_t kBlockSegmentWords = 32U;
constexpr std::uint8_t kBlockSegmentPayloadWords = 31U;  // word 0 of a segment is version and index
constexpr std::uint16_t kMaxBlockSegments = 256U;
constexpr std::uint16_t kMaxBlockPayloadWords = kMaxBlockSegments * kBlockSegmentPayloadWords;  // 7936

// CRC-16/CCITT-FALSE (polynomial 0x1021, initial value 0xFFFF, no reflection,
// no final XOR) over the words as big-endian bytes.
[[nodiscard]] std::uint16_t crc16_ccitt_false(const Word* words, std::uint16_t count) noexcept;

// Segments a payload of `length` words needs.
[[nodiscard]] constexpr std::uint16_t block_segment_count(std::uint16_t length) noexcept {
    return static_cast<std::uint16_t>((length + kBlockSegmentPayloadWords - 1U) / kBlockSegmentPayloadWords);
}

struct BlockDescriptor {
    std::uint8_t version{0};  // 0 = no block yet; otherwise 1..255, bumped with every payload
    std::uint16_t length{0};  // payload words
    std::uint16_t segments{0};
    std::uint16_t crc{0};
};

void encode_descriptor(const BlockDescriptor& d, Word* out) noexcept;  // kBlockDescriptorWords
void decode_descriptor(const Word* in, BlockDescriptor& d) noexcept;

class BlockTransmitter {
public:
    // `storage` holds the current payload: `capacity` words, at most kMaxBlockPayloadWords.
    [[nodiscard]] Status init(Word* storage, std::uint16_t capacity) noexcept;

    // Publishes a new payload: copies it in, bumps the version (1..255, never
    // 0) and starts again at segment 0. kBadLength if it does not fit.
    [[nodiscard]] Status set_payload(const Word* payload, std::uint16_t count) noexcept;

    [[nodiscard]] bool has_payload() const noexcept { return version_ != 0U; }
    [[nodiscard]] std::uint8_t version() const noexcept { return version_; }
    [[nodiscard]] std::uint16_t length() const noexcept { return length_; }
    [[nodiscard]] std::uint16_t segment_count() const noexcept { return segments_; }
    [[nodiscard]] std::uint16_t next_index() const noexcept { return next_; }

    void descriptor(Word* out) const noexcept;                    // kBlockDescriptorWords words
    void segment(std::uint16_t index, Word* out) const noexcept;  // kBlockSegmentWords words, zero-padded
    void next_segment(Word* out) const noexcept { segment(next_, out); }

    // The bus took the current segment: move to the next, wrapping to 0 after
    // the last so a consumer that joined late still gets every segment.
    void advance() noexcept;

private:
    Word* storage_{nullptr};
    std::uint16_t capacity_{0};
    std::uint16_t length_{0};
    std::uint16_t segments_{0};
    std::uint16_t crc_{0};
    std::uint16_t next_{0};
    std::uint8_t version_{0};
};

class BlockReceiver {
public:
    [[nodiscard]] Status init(Word* storage, std::uint16_t capacity) noexcept;

    // Each returns true when the block of the version being collected has just
    // become complete: every segment arrived and the CRC matches the
    // descriptor. The payload is then in the storage, length() words, until a
    // segment or descriptor of another version arrives. Version 0 (an unloaded
    // subaddress) is ignored.
    [[nodiscard]] bool feed_descriptor(const Word* in) noexcept;
    [[nodiscard]] bool feed_segment(const Word* in) noexcept;

    [[nodiscard]] bool complete() const noexcept { return complete_; }
    [[nodiscard]] std::uint8_t version() const noexcept { return collecting_; }
    [[nodiscard]] std::uint16_t length() const noexcept { return descriptor_.length; }
    [[nodiscard]] const Word* payload() const noexcept { return storage_; }
    [[nodiscard]] std::uint32_t completed_blocks() const noexcept { return completed_blocks_; }

private:
    void restart(std::uint8_t new_version) noexcept;
    [[nodiscard]] bool check_complete() noexcept;

    Word* storage_{nullptr};
    std::uint16_t capacity_{0};
    BlockDescriptor descriptor_{};
    bool have_descriptor_{false};
    std::uint8_t collecting_{0};
    std::array<std::uint32_t, kMaxBlockSegments / 32U> received_{};
    bool complete_{false};
    std::uint32_t completed_blocks_{0};
};

// A producer's block on its two transmit subaddresses: publish() a payload,
// then serve() it every frame. The segment on offer moves on each time the
// terminal's transmit count for the segment subaddress shows the bus took it.
class BlockPublisher {
public:
    [[nodiscard]] Status init(Word* storage, std::uint16_t capacity) noexcept;

    // A new payload, served from its first segment.
    [[nodiscard]] Status publish(const RemoteTerminal& terminal, std::uint8_t segment_sa, const Word* payload,
                                 std::uint16_t count) noexcept;

    // Every frame after the terminal has handled the bus: advance past a segment
    // the bus took, then load the descriptor and the segment to offer next.
    [[nodiscard]] Status serve(RemoteTerminal& terminal, std::uint8_t descriptor_sa, std::uint8_t segment_sa) noexcept;

    [[nodiscard]] std::uint8_t version() const noexcept { return tx_.version(); }
    [[nodiscard]] const BlockTransmitter& transmitter() const noexcept { return tx_; }

private:
    BlockTransmitter tx_{};
    std::uint32_t segment_reads_{0};
    bool published_{false};
};

// A consumer's block on its two receive subaddresses (the far end of a block
// relay): collect() feeds whatever arrived since the last call to the receiver
// and is true when a block has just completed.
class BlockCollector {
public:
    [[nodiscard]] Status init(Word* storage, std::uint16_t capacity) noexcept;
    [[nodiscard]] bool collect(RemoteTerminal& terminal, std::uint8_t descriptor_sa, std::uint8_t segment_sa) noexcept;
    [[nodiscard]] const BlockReceiver& receiver() const noexcept { return rx_; }

private:
    BlockReceiver rx_{};
};

}  // namespace fmofp::bus

#endif
