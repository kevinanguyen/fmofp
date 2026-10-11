#ifndef FMOFP_BC_SCHEDULE_HPP
#define FMOFP_BC_SCHEDULE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/status.hpp>
#include <fmofp/bus/word.hpp>

// The bus controller's fixed timeline (ICD README section 6): a 20 ms minor
// frame, 50 of them to the 1 s major frame, and every periodic message laid
// into the minor frames its rate requires. Built once at start-up; nothing
// about it changes in flight.

namespace fmofp::bc {

constexpr std::uint8_t kMinorFramesPerMajor = 50;
constexpr std::int64_t kMinorFramePeriodNs = 20 * 1000 * 1000;
constexpr std::uint16_t kMajorFrameModulus = 1024;  // the sync word carries 10 bits of it
constexpr std::size_t kMaxSlotsPerFrame = 32;
constexpr std::size_t kMaxScheduledMessages = 160;

// Bus time on a real 1 Mbit/s bus: 20 us per word, about 12 us of gaps per
// message. README section 6 caps a minor frame at 50% of 20 ms.
constexpr std::uint32_t kUsPerWord = 20;
constexpr std::uint32_t kUsPerMessageGap = 12;
constexpr std::uint32_t kMinorFrameBudgetUs = 10 * 1000;

// A producer may be relayed to this many consumers (icd_tool.py MAX_RELAYS_PER_SOURCE).
constexpr std::size_t kMaxRelaysPerMessage = 4;

// One consumer of a relayed transmit message: the terminal and subaddress
// that receive the same words straight after the poll.
struct RelayTarget {
    std::uint8_t rt{0};
    std::uint8_t subaddress{0};
};

struct ScheduledMessage {
    std::uint8_t rt{0};
    std::uint8_t subaddress{0};
    bool transmit{false};  // T: the terminal sends; R: the BC sends it data
    std::uint8_t words{0};
    std::uint8_t rate_hz{0};  // 50, 10, 5 or 1
    // For a transmit message that is relayed (ICD relays section): its
    // consumers, relay_count of them.
    std::array<RelayTarget, kMaxRelaysPerMessage> relays{};
    std::uint8_t relay_count{0};
};

struct Slot {
    std::uint8_t rt{0};
    std::uint8_t subaddress{0};
    bool transmit{false};
    std::uint8_t words{0};
    std::array<RelayTarget, kMaxRelaysPerMessage> relays{};
    std::uint8_t relay_count{0};

    [[nodiscard]] constexpr bool has_relay() const noexcept { return relay_count != 0U; }
};

struct FrameSlots {
    std::array<Slot, kMaxSlotsPerFrame> slots{};
    std::uint8_t count{0};
};

class Schedule {
public:
    // kInvalidArgument for a rate other than 50/10/5/1, a word count outside
    // 1..32 or an address out of range; kBadLength when the table is full.
    [[nodiscard]] Status add(const ScheduledMessage& message) noexcept;

    // Lays the added messages into the minor frames. Messages of one rate are
    // spread across the phases of their period so no frame carries them all.
    // kBadLength if a frame would exceed kMaxSlotsPerFrame.
    [[nodiscard]] Status build() noexcept;

    [[nodiscard]] bool built() const noexcept { return built_; }
    [[nodiscard]] std::size_t message_count() const noexcept { return count_; }
    [[nodiscard]] const FrameSlots& frame(std::uint8_t minor) const noexcept;

    // Estimated bus time of one minor frame, sync included.
    [[nodiscard]] std::uint32_t frame_load_us(std::uint8_t minor) const noexcept;
    [[nodiscard]] std::uint32_t worst_frame_load_us() const noexcept;

private:
    std::array<ScheduledMessage, kMaxScheduledMessages> messages_{};
    std::size_t count_{0};
    std::array<FrameSlots, kMinorFramesPerMajor> frames_{};
    bool built_{false};
};

// The data word of SYNCHRONIZE_WITH_DATA: bits 15-6 the major frame count
// modulo 1024, bits 5-0 the minor frame 0..49.
bus::Word encode_sync_word(std::uint16_t major, std::uint8_t minor) noexcept;
void decode_sync_word(bus::Word w, std::uint16_t& major, std::uint8_t& minor) noexcept;

}  // namespace fmofp::bc

#endif
