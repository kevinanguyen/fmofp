#ifndef FMOFP_OSAL_CLOCK_HPP
#define FMOFP_OSAL_CLOCK_HPP

#include <cstdint>

#include <fmofp/bus/status.hpp>

// Monotonic time and absolute-deadline sleeping. Rate-group code never sleeps
// for a duration (that drifts by the work time each cycle); it sleeps until
// the next deadline computed from the previous one (ICD README section 6).

namespace fmofp::osal {

// Nanoseconds since an unspecified, fixed point before the process started.
// Signed so that deadlines can be subtracted without surprises.
using Nanoseconds = std::int64_t;

constexpr Nanoseconds kNsPerUs = 1000;
constexpr Nanoseconds kNsPerMs = 1000 * kNsPerUs;
constexpr Nanoseconds kNsPerS = 1000 * kNsPerMs;

// Monotonic, never goes backwards, unaffected by wall-clock changes.
Nanoseconds now() noexcept;

// Blocks until now() >= deadline. Returns immediately (kOk) for a deadline
// already past. kIoError only if the OS timer itself fails.
[[nodiscard]] Status sleep_until(Nanoseconds deadline) noexcept;

// Convenience for start-up and tests; not for rate-group code.
[[nodiscard]] Status sleep_for(Nanoseconds duration) noexcept;

}  // namespace fmofp::osal

#endif
