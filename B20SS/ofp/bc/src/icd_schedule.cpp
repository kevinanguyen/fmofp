#include <fmofp/bc/icd_schedule.hpp>

#include <fmofp/icd/messages.hpp>

namespace fmofp::bc {

namespace {

bool listed(const std::uint8_t* terminals, std::size_t count, std::uint8_t rt) noexcept {
    for (std::size_t i = 0; i < count; ++i) {
        if (terminals[i] == rt) {
            return true;
        }
    }
    return false;
}

Status add_one(Schedule& schedule, const icd::MessageInfo& info, std::uint8_t rt) noexcept {
    ScheduledMessage m;
    m.rt = rt;
    m.subaddress = info.subaddress;
    m.transmit = info.direction == icd::Direction::kTransmit;
    m.words = info.words;
    m.rate_hz = info.rate_hz;
    return schedule.add(m);
}

}  // namespace

Status add_icd_messages(Schedule& schedule, const std::uint8_t* terminals, std::size_t count) noexcept {
    if (terminals == nullptr && count != 0) {
        return Status::kInvalidArgument;
    }
    for (const icd::MessageInfo& info : icd::kMessages) {
        if (info.rate_hz == icd::kAperiodic) {
            continue;
        }
        if (info.rt == icd::kEveryTerminal) {
            for (std::size_t i = 0; i < count; ++i) {
                const Status s = add_one(schedule, info, terminals[i]);
                if (s != Status::kOk) {
                    return s;
                }
            }
        } else if (listed(terminals, count, info.rt)) {
            const Status s = add_one(schedule, info, info.rt);
            if (s != Status::kOk) {
                return s;
            }
        }
    }
    return Status::kOk;
}

}  // namespace fmofp::bc
