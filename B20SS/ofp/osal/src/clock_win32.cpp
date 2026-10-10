#include <windows.h>

#include <fmofp/osal/clock.hpp>

namespace fmofp::osal {

namespace {

LONGLONG qpc_frequency() noexcept {
    LARGE_INTEGER f{};
    QueryPerformanceFrequency(&f);  // cannot fail on XP or later
    return f.QuadPart;
}

// Windows waitable timers take their due time in 100 ns units; a negative
// value means relative to now.
constexpr Nanoseconds kNsPer100ns = 100;

}  // namespace

Nanoseconds now() noexcept {
    static const LONGLONG freq = qpc_frequency();
    LARGE_INTEGER c{};
    QueryPerformanceCounter(&c);
    // Split the division so the multiplication by 1e9 cannot overflow for
    // any realistic counter value.
    const LONGLONG whole = c.QuadPart / freq;
    const LONGLONG part = c.QuadPart % freq;
    return static_cast<Nanoseconds>(whole) * kNsPerS + static_cast<Nanoseconds>(part) * kNsPerS / freq;
}

Status sleep_until(Nanoseconds deadline) noexcept {
    if (now() >= deadline) {
        return Status::kOk;
    }
    // High-resolution timers (Windows 10 1803+) wake within tens of
    // microseconds; the plain timer falls back to the ~1 ms scheduler tick.
    HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                          TIMER_ALL_ACCESS);
    if (timer == nullptr) {
        timer = CreateWaitableTimerW(nullptr, TRUE, nullptr);
    }
    if (timer == nullptr) {
        return Status::kIoError;
    }

    Status result = Status::kOk;
    for (int attempt = 0; attempt < 1000; ++attempt) {
        const Nanoseconds remaining = deadline - now();
        if (remaining <= 0) {
            break;
        }
        LARGE_INTEGER due{};
        due.QuadPart = -((remaining + kNsPer100ns - 1) / kNsPer100ns);
        if (SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE) == 0) {
            result = Status::kIoError;
            break;
        }
        if (WaitForSingleObject(timer, INFINITE) != WAIT_OBJECT_0) {
            result = Status::kIoError;
            break;
        }
    }
    CloseHandle(timer);
    return result;
}

}  // namespace fmofp::osal
