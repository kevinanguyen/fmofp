#include <fmofp/bc/icd_schedule.hpp>

#include <algorithm>

#include <fmofp/bus/block.hpp>
#include <fmofp/bus/command_word.hpp>
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

// A relay is made when its consumer is present: the terminal itself, or for a
// broadcast (address 31) at least one of the terminals that accept it.
bool wanted(const std::uint8_t* terminals, std::size_t count, std::uint8_t to_rt,
            std::uint32_t listeners) noexcept {
    if (to_rt != bus::kBroadcastRt) {
        return listed(terminals, count, to_rt);
    }
    for (std::size_t i = 0; i < count; ++i) {
        if (((listeners >> terminals[i]) & 1U) != 0U) {
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

Status add_relay(ScheduledMessage& m, std::uint8_t rt, std::uint8_t sa) noexcept {
    if (m.relay_count >= kMaxRelaysPerMessage) {
        return Status::kBadLength;
    }
    m.relays[m.relay_count].rt = rt;
    m.relays[m.relay_count].subaddress = sa;
    ++m.relay_count;
    return Status::kOk;
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
                !wanted(terminals, count, relay.to_rt, relay.listeners)) {
                continue;
            }
            const Status s = add_relay(m, relay.to_rt, relay.to_subaddress);
            if (s != Status::kOk) {
                return s;
            }
        }
    }
    return schedule.add(m);
}

// Block streams (ICD README section 8): the descriptor and the segment
// subaddress are polled like any periodic transmit message, and a block relay
// forwards both to each consumer that is present.
Status add_block(Schedule& schedule, const icd::BlockInfo& block, const std::uint8_t* terminals,
                 std::size_t count) noexcept {
    ScheduledMessage descriptor;
    descriptor.rt = block.rt;
    descriptor.subaddress = block.descriptor_subaddress;
    descriptor.transmit = true;
    descriptor.words = bus::kBlockDescriptorWords;
    descriptor.rate_hz = block.rate_hz;
    ScheduledMessage segment = descriptor;
    segment.subaddress = block.segment_subaddress;
    segment.words = bus::kBlockSegmentWords;
    for (const icd::BlockRelayInfo& relay : icd::kBlockRelays) {
        if (relay.from_rt != block.rt || relay.from_descriptor_subaddress != block.descriptor_subaddress ||
            !wanted(terminals, count, relay.to_rt, relay.listeners)) {
            continue;
        }
        const Status d = add_relay(descriptor, relay.to_rt, relay.to_descriptor_subaddress);
        const Status s = add_relay(segment, relay.to_rt, relay.to_segment_subaddress);
        if (d != Status::kOk || s != Status::kOk) {
            return Status::kBadLength;
        }
    }
    const Status d = schedule.add(descriptor);
    if (d != Status::kOk) {
        return d;
    }
    return schedule.add(segment);
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
    // A receive block is the far end of a block relay: delivered, never polled.
    for (const icd::BlockInfo& block : icd::kBlocks) {
        if (block.direction != icd::Direction::kTransmit || !listed(terminals, count, block.rt)) {
            continue;
        }
        const Status s = add_block(schedule, block, terminals, count);
        if (s != Status::kOk) {
            return s;
        }
    }
    return Status::kOk;
}

}  // namespace fmofp::bc
