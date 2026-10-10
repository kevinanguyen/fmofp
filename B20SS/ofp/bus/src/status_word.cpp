#include <fmofp/bus/command_word.hpp>
#include <fmofp/bus/status_word.hpp>

namespace fmofp::bus {

namespace {

constexpr unsigned kRtHi = 15U;
constexpr unsigned kRtLo = 11U;
constexpr unsigned kMessageErrorBit = 10U;
constexpr unsigned kInstrumentationBit = 9U;
constexpr unsigned kServiceRequestBit = 8U;
constexpr unsigned kReservedHi = 7U;
constexpr unsigned kReservedLo = 5U;
constexpr unsigned kBroadcastReceivedBit = 4U;
constexpr unsigned kBusyBit = 3U;
constexpr unsigned kSubsystemFlagBit = 2U;
constexpr unsigned kDynamicBusAcceptanceBit = 1U;
constexpr unsigned kTerminalFlagBit = 0U;

}  // namespace

Status encode_status_word(const StatusWord& sw, Word& out) noexcept {
    out = 0;
    if (sw.rt > kMaxRt) {
        return Status::kInvalidArgument;
    }
    Word w = 0;
    w = set_field(w, kRtHi, kRtLo, sw.rt);
    w = set_bit(w, kMessageErrorBit, sw.message_error);
    w = set_bit(w, kServiceRequestBit, sw.service_request);
    w = set_bit(w, kBroadcastReceivedBit, sw.broadcast_received);
    w = set_bit(w, kBusyBit, sw.busy);
    w = set_bit(w, kSubsystemFlagBit, sw.subsystem_flag);
    w = set_bit(w, kTerminalFlagBit, sw.terminal_flag);
    out = w;
    return Status::kOk;
}

Status decode_status_word(Word in, StatusWord& out) noexcept {
    out.rt = static_cast<std::uint8_t>(get_field(in, kRtHi, kRtLo));
    out.message_error = get_bit(in, kMessageErrorBit);
    out.service_request = get_bit(in, kServiceRequestBit);
    out.broadcast_received = get_bit(in, kBroadcastReceivedBit);
    out.busy = get_bit(in, kBusyBit);
    out.subsystem_flag = get_bit(in, kSubsystemFlagBit);
    out.terminal_flag = get_bit(in, kTerminalFlagBit);

    const bool reserved_set = get_field(in, kReservedHi, kReservedLo) != 0U ||
                              get_bit(in, kInstrumentationBit) ||
                              get_bit(in, kDynamicBusAcceptanceBit);
    return reserved_set ? Status::kMalformed : Status::kOk;
}

}  // namespace fmofp::bus
