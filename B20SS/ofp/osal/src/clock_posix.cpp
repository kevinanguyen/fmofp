#include <cerrno>
#include <ctime>

#include <fmofp/osal/clock.hpp>

namespace fmofp::osal {

namespace {

constexpr Nanoseconds kNsPerSecond = kNsPerS;

timespec to_timespec(Nanoseconds t) noexcept {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(t / kNsPerSecond);
    ts.tv_nsec = static_cast<long>(t % kNsPerSecond);
    return ts;
}

}  // namespace

Nanoseconds now() noexcept {
    timespec ts{};
    // CLOCK_MONOTONIC is the same clock clock_nanosleep waits against below,
    // so a deadline from now() is exact there, with no conversion.
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }
    return static_cast<Nanoseconds>(ts.tv_sec) * kNsPerSecond + static_cast<Nanoseconds>(ts.tv_nsec);
}

Status sleep_until(Nanoseconds deadline) noexcept {
    const timespec ts = to_timespec(deadline);
    // A signal can interrupt the wait early; TIMER_ABSTIME makes resuming
    // with the same deadline correct rather than cumulative.
    for (int attempt = 0; attempt < 1000; ++attempt) {
        const int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
        if (rc == 0) {
            return Status::kOk;
        }
        if (rc != EINTR) {
            return Status::kIoError;
        }
    }
    return Status::kIoError;
}

}  // namespace fmofp::osal
