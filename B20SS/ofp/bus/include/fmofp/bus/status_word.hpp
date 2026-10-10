#ifndef FMOFP_BUS_STATUS_WORD_HPP
#define FMOFP_BUS_STATUS_WORD_HPP

#include <cstdint>

#include <fmofp/bus/status.hpp>
#include <fmofp/bus/word.hpp>

// MIL-STD-1553B status word (ICD README section 3):
//   bits 15-11  RT address
//   bit  10     message error
//   bit  9      instrumentation (always 0)
//   bit  8      service request
//   bits 7-5    reserved (0)
//   bit  4      broadcast command received
//   bit  3      busy
//   bit  2      subsystem flag
//   bit  1      dynamic bus control acceptance (always 0)
//   bit  0      terminal flag

namespace fmofp::bus {

struct StatusWord {
    std::uint8_t rt{0};
    bool message_error{false};
    bool service_request{false};
    bool broadcast_received{false};
    bool busy{false};
    bool subsystem_flag{false};
    bool terminal_flag{false};
};

// kInvalidArgument when rt is above 31. The always-zero bits are not
// representable in StatusWord, so they are always sent as 0.
[[nodiscard]] Status encode_status_word(const StatusWord& sw, Word& out) noexcept;

// kMalformed when a reserved or always-zero bit is set; the other fields are
// still decoded so the bus controller can count the fault and carry on.
[[nodiscard]] Status decode_status_word(Word in, StatusWord& out) noexcept;

}  // namespace fmofp::bus

#endif
