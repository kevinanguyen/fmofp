#include <fmofp/osal/clock.hpp>

namespace fmofp::osal {

Status sleep_for(Nanoseconds duration) noexcept {
    if (duration <= 0) {
        return Status::kOk;
    }
    return sleep_until(now() + duration);
}

}  // namespace fmofp::osal
