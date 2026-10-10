#ifndef FMOFP_BC_ICD_SCHEDULE_HPP
#define FMOFP_BC_ICD_SCHEDULE_HPP

#include <cstddef>
#include <cstdint>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/status.hpp>

namespace fmofp::bc {

// Adds every periodic message of the ICD (icd::kMessages) for the given live
// terminals: messages carried by every terminal (TERM_STATUS) are added once
// per terminal in the list; other messages only if their terminal is in it.
// Aperiodic messages are not scheduled; the BC sends them on demand.
[[nodiscard]] Status add_icd_messages(Schedule& schedule, const std::uint8_t* terminals,
                                      std::size_t count) noexcept;

}  // namespace fmofp::bc

#endif
