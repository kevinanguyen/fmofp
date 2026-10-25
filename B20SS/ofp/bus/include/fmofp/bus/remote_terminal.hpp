#ifndef FMOFP_BUS_REMOTE_TERMINAL_HPP
#define FMOFP_BUS_REMOTE_TERMINAL_HPP

#include <array>
#include <cstdint>

#include <fmofp/bus/command_word.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/bus/transport.hpp>
#include <fmofp/bus/word.hpp>

// What every terminal owes the bus, independent of which subsystem it is:
// fixed buffers per subaddress, the status word, and the mode codes of the
// ICD (ICD_TABLES.md, Mode codes). A partition owns one RemoteTerminal, loads
// its transmit subaddresses each frame, reads its receive subaddresses, and
// hands every datagram from the bus controller to handle().
//
// Errors on the bus are never exceptions and never stop the terminal: a bad
// command is answered with silence and the message-error bit in the next
// status word, which is how a real RT behaves.

namespace fmofp::bus {

constexpr std::uint8_t kModeCodeTransmitStatusWord = 2;
constexpr std::uint8_t kModeCodeInitiateSelfTest = 3;
constexpr std::uint8_t kModeCodeInhibitTerminalFlag = 6;
constexpr std::uint8_t kModeCodeOverrideInhibitTerminalFlag = 7;
constexpr std::uint8_t kModeCodeResetRemoteTerminal = 8;
constexpr std::uint8_t kModeCodeTransmitVectorWord = 16;
constexpr std::uint8_t kModeCodeSynchronizeWithData = 17;
constexpr std::uint8_t kModeCodeTransmitLastCommand = 18;
constexpr std::uint8_t kModeCodeTransmitBitWord = 19;

constexpr std::uint8_t kFirstDataSubaddress = 1;
constexpr std::uint8_t kLastDataSubaddress = 30;

// The built-in-test word: the layout of TERM_STATUS word 2 and of the data
// word returned by mode code 19.
struct BitWord {
    bool in_progress{false};
    bool failed{false};  // latched until RESET_REMOTE_TERMINAL
    std::uint8_t fault_count{0};
};

Word encode_bit_word(const BitWord& bit) noexcept;
BitWord decode_bit_word(Word w) noexcept;

// ICD README section 5: a receiver acts on a command only when its cmd_seq
// differs from the last one it processed, so resending is harmless. cmd_seq 0
// is no command at all: a command subaddress nobody has loaded reads as zeros,
// and the bus controller relays those like anything else.
struct CommandSequence {
    bool have_last{false};
    std::uint8_t last{0};

    // True if `seq` is a new command; records it then.
    bool accept(std::uint8_t seq) noexcept {
        if (seq == 0U || (have_last && seq == last)) {
            return false;
        }
        have_last = true;
        last = seq;
        return true;
    }
};

struct RtCounters {
    std::uint32_t commands{0};        // addressed to this terminal or broadcast
    std::uint32_t ignored{0};         // addressed to another terminal
    std::uint32_t receives{0};        // data messages accepted
    std::uint32_t transmits{0};       // data messages answered
    std::uint32_t broadcasts{0};          // broadcast data and mode codes acted on
    std::uint32_t broadcasts_ignored{0};  // broadcast data on a subaddress not accepted
    std::uint32_t syncs{0};
    std::uint32_t message_errors{0};  // bad commands, answered with silence
};

class RemoteTerminal {
public:
    static constexpr std::uint8_t kMaxWordsPerSubaddress = kMaxDataWords;

    // Address 0..30 (31 is broadcast). Clears every buffer and flag.
    [[nodiscard]] Status init(std::uint8_t rt) noexcept;
    [[nodiscard]] std::uint8_t address() const noexcept { return rt_; }

    // ---- the partition's side ---------------------------------------------

    // Loads what the terminal will send the next time the bus controller
    // asks for subaddress `sa` (1..30, 1..32 words). Unloaded subaddresses
    // transmit zeros, the fail-safe pattern.
    [[nodiscard]] Status set_transmit(std::uint8_t sa, const Word* words, std::uint8_t count) noexcept;

    // The last data received on `sa`. kNotReady until something has arrived;
    // kBadLength if `capacity` is too small. `fresh` is true if it arrived
    // since the previous call for that subaddress.
    [[nodiscard]] Status get_received(std::uint8_t sa, Word* words, std::uint8_t capacity,
                                      std::uint8_t& count, bool& fresh) noexcept;

    // Raises the service request bit; the vector word (mode code 16) names `sa`.
    void request_service(std::uint8_t sa) noexcept;
    void set_subsystem_flag(bool set) noexcept { subsystem_flag_ = set; }
    void set_terminal_flag(bool set) noexcept { terminal_flag_ = set; }
    void set_bit(const BitWord& bit) noexcept { bit_ = bit; }
    [[nodiscard]] const BitWord& bit() const noexcept { return bit_; }

    // Each returns true once per event, then false until the next one. The
    // partition polls them after handling the frame's datagrams.
    bool take_sync(Word& sync_word) noexcept;    // SYNCHRONIZE_WITH_DATA arrived
    bool take_reset_request() noexcept;          // RESET_REMOTE_TERMINAL arrived
    bool take_self_test_request() noexcept;      // INITIATE_SELF_TEST arrived

    [[nodiscard]] const RtCounters& counters() const noexcept { return counters_; }

    // How many times the bus controller has read subaddress `sa` since init
    // or reset. A block transmitter watches this to know when the bus took
    // the segment it loaded (bus/block.hpp).
    [[nodiscard]] std::uint32_t transmit_count(std::uint8_t sa) const noexcept {
        return sa <= kMaxSubaddress ? tx_count_[sa] : 0U;
    }

    // Broadcast data (a receive command to address 31) is stored only on the
    // subaddresses the partition accepts here (ICD README section 8: the
    // broadcast map); on any other it is ignored, so a broadcast can never
    // land in a buffer meant for this terminal's own commands. Mode codes
    // such as the sync need no acceptance. Kept across RESET_REMOTE_TERMINAL;
    // init() clears it.
    [[nodiscard]] Status accept_broadcast(std::uint8_t sa) noexcept;
    [[nodiscard]] bool accepts_broadcast(std::uint8_t sa) const noexcept {
        return sa <= kMaxSubaddress && ((broadcast_accept_ >> sa) & 1U) != 0U;
    }

    // ---- the bus side -----------------------------------------------------

    // Processes one datagram from the bus controller. On return `reply_needed`
    // says whether `reply` must be sent back: it is false for a broadcast, a
    // frame flagged with a parity error, an invalid command (message error
    // latched for the next status word) and a command for another terminal.
    // Returns kInvalidArgument for a frame that is not BC-to-RT and
    // kBadLength for one with no command word; both are transport faults,
    // not bus messages, and get no reply either.
    [[nodiscard]] Status handle(const Frame& in, Frame& reply, bool& reply_needed) noexcept;

private:
    // Builds the status word. The message-error and broadcast-received bits
    // mean "since the last status word", so they clear once one is built to
    // send, except for TRANSMIT_STATUS_WORD, which re-reads the last one.
    Word status_word(bool clear_latches) noexcept;
    void reply_status(const Frame& in, Frame& reply, bool clear_latches) noexcept;
    void handle_receive(const CommandWord& cw, const Frame& in, Frame& reply, bool& reply_needed) noexcept;
    void handle_transmit(const CommandWord& cw, const Frame& in, Frame& reply, bool& reply_needed) noexcept;
    void handle_mode_code(const CommandWord& cw, const Frame& in, Frame& reply, bool& reply_needed) noexcept;
    void message_error() noexcept;
    void reset() noexcept;

    std::uint8_t rt_{0};
    std::array<std::array<Word, kMaxDataWords>, kMaxSubaddress + 1> rx_{};
    std::array<std::array<Word, kMaxDataWords>, kMaxSubaddress + 1> tx_{};
    std::array<std::uint8_t, kMaxSubaddress + 1> rx_count_{};
    std::array<bool, kMaxSubaddress + 1> rx_valid_{};
    std::array<bool, kMaxSubaddress + 1> rx_fresh_{};
    std::array<std::uint32_t, kMaxSubaddress + 1> tx_count_{};
    std::uint32_t broadcast_accept_{0};  // bit per subaddress
    Word previous_command_{0};
    Word last_command_{0};
    Word sync_word_{0};
    std::uint8_t service_sa_{0};
    bool message_error_{false};
    bool broadcast_received_{false};
    bool service_requested_{false};
    bool subsystem_flag_{false};
    bool terminal_flag_{false};
    bool terminal_flag_inhibited_{false};
    bool sync_pending_{false};
    bool reset_pending_{false};
    bool self_test_pending_{false};
    BitWord bit_{};
    RtCounters counters_{};
};

}  // namespace fmofp::bus

#endif
