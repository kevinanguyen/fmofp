#include <fmofp/bus/remote_terminal.hpp>

#include <algorithm>
#include <cstddef>
#include <iterator>

#include <fmofp/bus/status_word.hpp>

namespace fmofp::bus {

namespace {

constexpr unsigned kBitInProgressBit = 15U;
constexpr unsigned kBitFailedBit = 14U;
constexpr unsigned kBitFaultCountHi = 7U;
constexpr unsigned kBitFaultCountLo = 0U;

// Direction, data word and broadcast rules per mode code (ICD mode code table
// and MIL-STD-1553B: codes 0-15 carry no data word, 16-31 carry one).
struct ModeCodeRule {
    std::uint8_t code;
    bool transmit;        // T/R bit the command must carry
    bool data_in;         // the command is followed by one data word
    bool data_out;        // the reply carries one data word
    bool broadcast_ok;
};

constexpr ModeCodeRule kModeCodeRules[] = {
    {kModeCodeTransmitStatusWord, true, false, false, false},
    {kModeCodeInitiateSelfTest, true, false, false, true},
    {kModeCodeInhibitTerminalFlag, true, false, false, true},
    {kModeCodeOverrideInhibitTerminalFlag, true, false, false, true},
    {kModeCodeResetRemoteTerminal, true, false, false, true},
    {kModeCodeTransmitVectorWord, true, false, true, false},
    {kModeCodeSynchronizeWithData, false, true, false, true},
    {kModeCodeTransmitLastCommand, true, false, true, false},
    {kModeCodeTransmitBitWord, true, false, true, false},
};

const ModeCodeRule* find_rule(std::uint8_t code) noexcept {
    const auto* const end = std::end(kModeCodeRules);
    const auto* const it = std::find_if(std::begin(kModeCodeRules), end,
                                        [code](const ModeCodeRule& rule) { return rule.code == code; });
    return it == end ? nullptr : it;
}

}  // namespace

Word encode_bit_word(const BitWord& bit) noexcept {
    Word w = 0;
    w = set_bit(w, kBitInProgressBit, bit.in_progress);
    w = set_bit(w, kBitFailedBit, bit.failed);
    w = set_field(w, kBitFaultCountHi, kBitFaultCountLo, bit.fault_count);
    return w;
}

BitWord decode_bit_word(Word w) noexcept {
    BitWord bit;
    bit.in_progress = get_bit(w, kBitInProgressBit);
    bit.failed = get_bit(w, kBitFailedBit);
    bit.fault_count = static_cast<std::uint8_t>(get_field(w, kBitFaultCountHi, kBitFaultCountLo));
    return bit;
}

Status RemoteTerminal::init(std::uint8_t rt) noexcept {
    if (rt >= kBroadcastRt) {
        return Status::kInvalidArgument;
    }
    rt_ = rt;
    broadcast_accept_ = 0;
    reset();
    reset_pending_ = false;
    counters_ = RtCounters{};
    return Status::kOk;
}

Status RemoteTerminal::accept_broadcast(std::uint8_t sa) noexcept {
    if (sa < kFirstDataSubaddress || sa > kLastDataSubaddress) {
        return Status::kInvalidArgument;
    }
    broadcast_accept_ |= 1U << sa;
    return Status::kOk;
}

void RemoteTerminal::reset() noexcept {
    rx_ = {};
    tx_ = {};
    rx_count_ = {};
    rx_valid_ = {};
    rx_fresh_ = {};
    tx_count_ = {};
    previous_command_ = 0;
    last_command_ = 0;
    sync_word_ = 0;
    service_sa_ = 0;
    message_error_ = false;
    broadcast_received_ = false;
    service_requested_ = false;
    subsystem_flag_ = false;
    terminal_flag_ = false;
    terminal_flag_inhibited_ = false;
    sync_pending_ = false;
    self_test_pending_ = false;
    bit_ = BitWord{};
    // reset_pending_ is left alone: the partition still has to learn that a
    // RESET_REMOTE_TERMINAL happened.
}

Status RemoteTerminal::set_transmit(std::uint8_t sa, const Word* words, std::uint8_t count) noexcept {
    if (sa < kFirstDataSubaddress || sa > kLastDataSubaddress || words == nullptr || count < 1 ||
        count > kMaxDataWords) {
        return Status::kInvalidArgument;
    }
    for (std::size_t i = 0; i < kMaxDataWords; ++i) {
        tx_[sa][i] = i < count ? words[i] : Word{0};
    }
    return Status::kOk;
}

Status RemoteTerminal::get_received(std::uint8_t sa, Word* words, std::uint8_t capacity,
                                    std::uint8_t& count, bool& fresh) noexcept {
    count = 0;
    fresh = false;
    if (sa < kFirstDataSubaddress || sa > kLastDataSubaddress || words == nullptr) {
        return Status::kInvalidArgument;
    }
    if (!rx_valid_[sa]) {
        return Status::kNotReady;
    }
    if (capacity < rx_count_[sa]) {
        return Status::kBadLength;
    }
    for (std::size_t i = 0; i < rx_count_[sa]; ++i) {
        words[i] = rx_[sa][i];
    }
    count = rx_count_[sa];
    fresh = rx_fresh_[sa];
    rx_fresh_[sa] = false;
    return Status::kOk;
}

void RemoteTerminal::request_service(std::uint8_t sa) noexcept {
    service_requested_ = true;
    service_sa_ = static_cast<std::uint8_t>(sa & kMaxSubaddress);
}

bool RemoteTerminal::take_sync(Word& sync_word) noexcept {
    if (!sync_pending_) {
        return false;
    }
    sync_pending_ = false;
    sync_word = sync_word_;
    return true;
}

bool RemoteTerminal::take_reset_request() noexcept {
    const bool pending = reset_pending_;
    reset_pending_ = false;
    return pending;
}

bool RemoteTerminal::take_self_test_request() noexcept {
    const bool pending = self_test_pending_;
    self_test_pending_ = false;
    return pending;
}

Word RemoteTerminal::status_word(bool clear_latches) noexcept {
    StatusWord sw;
    sw.rt = rt_;
    sw.message_error = message_error_;
    sw.service_request = service_requested_;
    sw.broadcast_received = broadcast_received_;
    sw.busy = false;
    sw.subsystem_flag = subsystem_flag_;
    sw.terminal_flag = terminal_flag_ && !terminal_flag_inhibited_;
    Word w = 0;
    if (encode_status_word(sw, w) != Status::kOk) {
        // Unreachable: init() refuses any address the status word cannot carry.
        return 0;
    }
    if (clear_latches) {
        message_error_ = false;
        broadcast_received_ = false;
    }
    return w;
}

void RemoteTerminal::reply_status(const Frame& in, Frame& reply, bool clear_latches) noexcept {
    reply = Frame{};
    reply.type = FrameType::kRtToBc;
    reply.flags = 0;
    reply.transaction = in.transaction;
    reply.word_count = 1;
    reply.words[0] = status_word(clear_latches);
}

void RemoteTerminal::message_error() noexcept {
    message_error_ = true;
    ++counters_.message_errors;
}

Status RemoteTerminal::handle(const Frame& in, Frame& reply, bool& reply_needed) noexcept {
    reply_needed = false;
    if (in.type != FrameType::kBcToRt) {
        return Status::kInvalidArgument;
    }
    if (in.word_count < 1) {
        return Status::kBadLength;
    }
    const CommandWord cw = decode_command_word(in.words[0]);
    if (cw.rt != rt_ && !cw.is_broadcast()) {
        ++counters_.ignored;
        return Status::kOk;
    }
    ++counters_.commands;
    previous_command_ = last_command_;
    last_command_ = in.words[0];

    // A parity error is detected by the terminal, which then behaves as if
    // the message never arrived, except for remembering that it was bad.
    if ((in.flags & kFlagParityError) != 0U) {
        message_error();
        return Status::kOk;
    }

    if (cw.is_mode_code()) {
        handle_mode_code(cw, in, reply, reply_needed);
    } else if (cw.transmit) {
        handle_transmit(cw, in, reply, reply_needed);
    } else {
        handle_receive(cw, in, reply, reply_needed);
    }
    return Status::kOk;
}

void RemoteTerminal::handle_receive(const CommandWord& cw, const Frame& in, Frame& reply,
                                    bool& reply_needed) noexcept {
    if (cw.is_broadcast() && !accepts_broadcast(cw.subaddress)) {
        ++counters_.broadcasts_ignored;  // not in this terminal's broadcast map: not stored, not flagged
        return;
    }
    const std::size_t expected = 1U + cw.count_or_mode;
    if (in.word_count != expected) {
        message_error();
        return;
    }
    for (std::size_t i = 0; i < kMaxDataWords; ++i) {
        rx_[cw.subaddress][i] = i < cw.count_or_mode ? in.words[1 + i] : Word{0};
    }
    rx_count_[cw.subaddress] = cw.count_or_mode;
    rx_valid_[cw.subaddress] = true;
    rx_fresh_[cw.subaddress] = true;
    ++counters_.receives;

    if (cw.is_broadcast()) {
        broadcast_received_ = true;
        ++counters_.broadcasts;
        return;
    }
    reply_status(in, reply, true);
    reply_needed = true;
}

void RemoteTerminal::handle_transmit(const CommandWord& cw, const Frame& in, Frame& reply,
                                     bool& reply_needed) noexcept {
    // Nobody can answer a broadcast, and a transmit command carries no data.
    if (cw.is_broadcast() || in.word_count != 1) {
        message_error();
        return;
    }
    reply_status(in, reply, true);
    for (std::size_t i = 0; i < cw.count_or_mode; ++i) {
        reply.words[1 + i] = tx_[cw.subaddress][i];
    }
    reply.word_count = static_cast<std::uint8_t>(1U + cw.count_or_mode);
    ++counters_.transmits;
    ++tx_count_[cw.subaddress];
    reply_needed = true;
}

void RemoteTerminal::handle_mode_code(const CommandWord& cw, const Frame& in, Frame& reply,
                                      bool& reply_needed) noexcept {
    const ModeCodeRule* rule = find_rule(cw.count_or_mode);
    const bool has_data = in.word_count == 2;
    if (rule == nullptr || cw.transmit != rule->transmit || has_data != rule->data_in ||
        in.word_count > 2 || (cw.is_broadcast() && !rule->broadcast_ok)) {
        message_error();
        return;
    }

    switch (rule->code) {
        case kModeCodeInitiateSelfTest:
            self_test_pending_ = true;
            bit_.in_progress = true;
            break;
        case kModeCodeInhibitTerminalFlag:
            terminal_flag_inhibited_ = true;
            break;
        case kModeCodeOverrideInhibitTerminalFlag:
            terminal_flag_inhibited_ = false;
            break;
        case kModeCodeResetRemoteTerminal:
            reset();
            last_command_ = in.words[0];
            reset_pending_ = true;
            break;
        case kModeCodeSynchronizeWithData:
            sync_word_ = in.words[1];
            sync_pending_ = true;
            ++counters_.syncs;
            break;
        default:
            break;  // the transmit-only codes change nothing
    }

    if (cw.is_broadcast()) {
        broadcast_received_ = true;
        ++counters_.broadcasts;
        return;
    }

    reply_status(in, reply, rule->code != kModeCodeTransmitStatusWord);
    if (rule->data_out) {
        Word data = 0;
        switch (rule->code) {
            case kModeCodeTransmitVectorWord:
                // The status word just built still shows the request; it is
                // satisfied once the vector word has gone out.
                data = service_sa_;
                service_requested_ = false;
                break;
            case kModeCodeTransmitLastCommand:
                data = previous_command_;
                break;
            case kModeCodeTransmitBitWord:
                data = encode_bit_word(bit_);
                break;
            default:
                break;
        }
        reply.words[1] = data;
        reply.word_count = 2;
    }
    reply_needed = true;
}

}  // namespace fmofp::bus
