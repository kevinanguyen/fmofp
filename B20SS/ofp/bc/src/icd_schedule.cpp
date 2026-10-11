#include <fmofp/bc/icd_schedule.hpp>

#include <algorithm>

#include <fmofp/bus/block.hpp>
#include <fmofp/icd/blocks.hpp>
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
        // Relay to every consumer that is present; a consumer that is not
        // gets nothing, and the message is still polled.
        for (const icd::RelayInfo& relay : icd::kRelays) {
            if (relay.from_rt != rt || relay.from_subaddress != info.subaddress ||
                !listed(terminals, count, relay.to_rt)) {
                continue;
            }
            if (m.relay_count >= kMaxRelaysPerMessage) {
                return Status::kBadLength;
            }
            m.relays[m.relay_count].rt = relay.to_rt;
            m.relays[m.relay_count].subaddress = relay.to_subaddress;
            ++m.relay_count;
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
    // Block streams (ICD README section 8): the descriptor and the segment
    // subaddress are polled like any periodic transmit message.
    for (const icd::BlockInfo& block : icd::kBlocks) {
        if (!listed(terminals, count, block.rt)) {
            continue;
        }
        ScheduledMessage descriptor;
        descriptor.rt = block.rt;
        descriptor.subaddress = block.descriptor_subaddress;
        descriptor.transmit = true;
        descriptor.words = bus::kBlockDescriptorWords;
        descriptor.rate_hz = block.rate_hz;
        const Status d = schedule.add(descriptor);
        if (d != Status::kOk) {
            return d;
        }
        ScheduledMessage segment = descriptor;
        segment.subaddress = block.segment_subaddress;
        segment.words = bus::kBlockSegmentWords;
        const Status s = schedule.add(segment);
        if (s != Status::kOk) {
            return s;
        }
    }
    return Status::kOk;
}

}  // namespace fmofp::bc
