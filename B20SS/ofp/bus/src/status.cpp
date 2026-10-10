#include <fmofp/bus/status.hpp>

namespace fmofp {

const char* to_string(Status s) noexcept {
    switch (s) {
        case Status::kOk:
            return "ok";
        case Status::kClamped:
            return "clamped";
        case Status::kInvalidEnum:
            return "invalid enum";
        case Status::kInvalidArgument:
            return "invalid argument";
        case Status::kMalformed:
            return "malformed";
        case Status::kBadLength:
            return "bad length";
        case Status::kBadMagic:
            return "bad magic";
        case Status::kTimeout:
            return "timeout";
        case Status::kNotReady:
            return "not ready";
        case Status::kIoError:
            return "io error";
    }
    return "unknown status";
}

}  // namespace fmofp
