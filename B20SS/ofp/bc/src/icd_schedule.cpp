#include <fmofp/bc/icd_schedule.hpp>

#include <algorithm>

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

// The relay whose source is this transmit message, if any.
const icd::RelayInfo* relay_from(std::uint8_t rt, std::uint8_t sa) noexcept {
    const auto it = std::find_if(icd::kRelays.begin(), icd::kRelays.end(), [rt, sa](const icd::RelayInfo& relay) {
        return relay.from_rt == rt && relay.from_subaddress == sa;
    });
    return it == icd::kRelays.end() ? nullptr : &*it;
}

// A receive message that a relay delivers needs no slot of its own.
bool is_relay_target(std::uint8_t rt, std::uint8_t sa) noexcept {
    return std::any_of(icd::kRelays.begin(), icd::kRelays.end(), [rt, sa](const icd::RelayInfo& relay) {
        return relay.to_rt == rt && relay.to_subaddress == sa;
    });
}

Status add_one(Schedule& schedule, const icd::MessageInfo& info, std::uint8_t rt,
               const std::uint8_t* terminals, std::size_t count) noexcept {
    ScheduledMessage m;
    m.rt = rt;
    m.subaddress = info.subaddress;
    m.transmit = info.direction == icd::Direction::kTransmit;
    m.words = info.words;
    m.rate_hz = info.rate_hz;
    if (m.transmit) {
        // Relay only to a terminal that is present; otherwise just poll.
        const icd::RelayInfo* relay = relay_from(rt, info.subaddress);
        if (relay != nullptr && listed(terminals, count, relay->to_rt)) {
            m.relay_rt = relay->to_rt;
            m.relay_subaddress = relay->to_subaddress;
        }
    }
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
        if (info.direction == icd::Direction::kReceive && info.rt != icd::kEveryTerminal &&
            is_relay_target(info.rt, info.subaddress)) {
            continue;
        }
        if (info.rt == icd::kEveryTerminal) {
            for (std::size_t i = 0; i < count; ++i) {
                const Status s = add_one(schedule, info, terminals[i], terminals, count);
                if (s != Status::kOk) {
                    return s;
                }
            }
        } else if (listed(terminals, count, info.rt)) {
            const Status s = add_one(schedule, info, info.rt, terminals, count);
            if (s != Status::kOk) {
                return s;
            }
        }
    }
    return Status::kOk;
}

}  // namespace fmofp::bc
