#include <fmofp/bc/bus_controller.hpp>

#include <fmofp/bus/command_word.hpp>
#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status_word.hpp>

namespace fmofp::bc {

namespace {

std::uint16_t terminal_port(const BcConfig& c, std::uint8_t rt) noexcept {
    return static_cast<std::uint16_t>(c.port_base + rt);
}

std::uint16_t bc_port(const BcConfig& c) noexcept {
    return static_cast<std::uint16_t>(c.port_base + c.bc_port_offset);
}

std::uint16_t monitor_port(const BcConfig& c) noexcept {
    return static_cast<std::uint16_t>(c.port_base + c.monitor_port_offset);
}

}  // namespace

Status BusController::init(const BcConfig& cfg, const Schedule& schedule) noexcept {
    shutdown();
    if (!schedule.built() || cfg.terminal_count > kMaxTerminals || cfg.response_timeout <= 0) {
        return Status::kInvalidArgument;
    }
    for (std::uint8_t i = 0; i < cfg.terminal_count; ++i) {
        if (cfg.terminals[i] >= bus::kBroadcastRt) {
            return Status::kInvalidArgument;
        }
    }
    config_ = cfg;
    schedule_ = schedule;
    transaction_ = 0;
    major_ = 0;
    minor_ = 0;
    started_ = false;
    terminals_ = {};
    stats_ = BcStats{};
    latest_ = {};
    queued_ = 0;

    const Status s = osal::init();
    if (s != Status::kOk) {
        return s;
    }
    return socket_.open(bc_port(config_));
}

void BusController::shutdown() noexcept { socket_.close(); }

const TerminalStats& BusController::terminal(std::uint8_t rt) const noexcept {
    return terminals_[rt <= bus::kBroadcastRt ? rt : bus::kBroadcastRt];
}

Status BusController::queue_receive(std::uint8_t rt, std::uint8_t sa, const bus::Word* words,
                                    std::uint8_t count) noexcept {
    if (rt >= bus::kBroadcastRt || sa < bus::kFirstDataSubaddress || sa > bus::kLastDataSubaddress ||
        words == nullptr || count < 1 || count > bus::kMaxDataWords) {
        return Status::kInvalidArgument;
    }
    if (queued_ >= kAperiodicQueueDepth) {
        ++stats_.aperiodic_dropped;
        return Status::kNotReady;
    }
    Aperiodic& a = queue_[queued_];
    a.rt = rt;
    a.sa = sa;
    a.count = count;
    for (std::size_t i = 0; i < bus::kMaxDataWords; ++i) {
        a.words[i] = i < count ? words[i] : bus::Word{0};
    }
    ++queued_;
    return Status::kOk;
}

Status BusController::get_latest(std::uint8_t rt, std::uint8_t sa, bus::Word* out, std::uint8_t capacity,
                                 std::uint8_t& count, std::uint32_t& frame) const noexcept {
    count = 0;
    frame = 0;
    if (rt >= bus::kBroadcastRt || sa > bus::kMaxSubaddress || out == nullptr) {
        return Status::kInvalidArgument;
    }
    const LatestData& d = latest_[rt][sa];
    if (!d.valid) {
        return Status::kNotReady;
    }
    if (capacity < d.count) {
        return Status::kBadLength;
    }
    for (std::size_t i = 0; i < d.count; ++i) {
        out[i] = d.words[i];
    }
    count = d.count;
    frame = d.frame;
    return Status::kOk;
}

Status BusController::send_sync() noexcept {
    const bus::Word sync = encode_sync_word(major_, minor_);
    bus::Frame unused;
    return transact(bus::kBroadcastRt, false, bus::kMaxSubaddress, bus::kModeCodeSynchronizeWithData,
                    &sync, 1, false, unused);
}

Status BusController::run_minor_frame() noexcept {
    if (!socket_.is_open()) {
        return Status::kNotReady;
    }
    if (!started_) {
        next_deadline_ = osal::now();
        started_ = true;
    }
    if (config_.mode == BcMode::kRealTime) {
        const Status s = osal::sleep_until(next_deadline_);
        if (s != Status::kOk) {
            return s;
        }
    }
    const osal::Nanoseconds frame_start = osal::now();

    Status result = send_sync();

    const FrameSlots& slots = schedule_.frame(minor_);
    for (std::size_t i = 0; i < slots.count; ++i) {
        const Slot& slot = slots.slots[i];
        bus::Frame reply;
        // A receive slot with nothing queued for it carries the terminal's
        // last known data: zeros until the producer side exists (ICD v2).
        const LatestData& stored = latest_[slot.rt][slot.subaddress];
        const Status s = transact(slot.rt, slot.transmit, slot.subaddress, slot.words,
                                  slot.transmit ? nullptr : stored.words.data(),
                                  slot.transmit ? std::uint8_t{0} : slot.words, true, reply);
        result = worst(result, s);
        // A relayed message goes straight on to its consumer with the words
        // just received; if the producer did not answer, the consumer gets
        // nothing this frame and its copy ages out (stale after 3 periods).
        if (s == Status::kOk && slot.transmit && slot.has_relay()) {
            bus::Frame relay_reply;
            const Status r = transact(slot.relay_rt, false, slot.relay_subaddress, slot.words, &reply.words[1],
                                      slot.words, true, relay_reply);
            result = worst(result, r);
            ++stats_.relays;
        }
    }

    for (std::uint8_t i = 0; i < queued_; ++i) {
        const Aperiodic& a = queue_[i];
        bus::Frame reply;
        const Status s = transact(a.rt, false, a.sa, a.count, a.words.data(), a.count, true, reply);
        result = worst(result, s);
        ++stats_.aperiodic_sent;
    }
    queued_ = 0;

    ++stats_.frames;
    minor_ = static_cast<std::uint8_t>(minor_ + 1U);
    if (minor_ >= kMinorFramesPerMajor) {
        minor_ = 0;
        major_ = static_cast<std::uint16_t>((major_ + 1U) % kMajorFrameModulus);
    }

    const osal::Nanoseconds frame_end = osal::now();
    const osal::Nanoseconds took = frame_end - frame_start;
    if (took > stats_.worst_frame_ns) {
        stats_.worst_frame_ns = took;
    }
    next_deadline_ += kMinorFramePeriodNs;
    if (config_.mode == BcMode::kRealTime && frame_end > next_deadline_) {
        ++stats_.overruns;
    } else if (config_.mode == BcMode::kFreeRun) {
        // Free running has no timeline to fall behind; keep deadlines current
        // so a later switch to real time does not start with a backlog.
        next_deadline_ = frame_end;
    }
    return result;
}

Status BusController::run_frames(std::uint32_t count) noexcept {
    Status result = Status::kOk;
    for (std::uint32_t i = 0; i < count; ++i) {
        result = worst(result, run_minor_frame());
    }
    return result;
}

Status BusController::transact(std::uint8_t rt, bool transmit, std::uint8_t sa, std::uint8_t count_or_mode,
                               const bus::Word* data, std::uint8_t data_count, bool expect_reply,
                               bus::Frame& reply) noexcept {
    reply = bus::Frame{};
    if (!socket_.is_open() || data_count > bus::kMaxDataWords || (data_count > 0 && data == nullptr) ||
        1U + data_count > bus::kMaxFrameWords) {
        return Status::kInvalidArgument;
    }

    bus::CommandWord cw;
    cw.rt = rt;
    cw.transmit = transmit;
    cw.subaddress = sa;
    cw.count_or_mode = count_or_mode;

    bus::Frame command;
    command.type = bus::FrameType::kBcToRt;
    command.flags = 0;
    transaction_ = static_cast<bus::Word>(transaction_ + 1U);
    command.transaction = transaction_;
    const Status enc = bus::encode_command_word(cw, command.words[0]);
    if (enc != Status::kOk) {
        return enc;
    }
    for (std::size_t i = 0; i < data_count; ++i) {
        command.words[1 + i] = data[i];
    }
    command.word_count = static_cast<std::uint8_t>(1U + data_count);

    std::array<std::uint8_t, bus::kMaxFrameBytes> bytes{};
    std::size_t length = 0;
    const Status fenc = bus::encode_frame(command, bytes.data(), bytes.size(), length);
    if (fenc != Status::kOk) {
        return fenc;
    }

    if (cw.is_broadcast()) {
        Status result = Status::kOk;
        for (std::uint8_t i = 0; i < config_.terminal_count; ++i) {
            const Status s = socket_.send_to(terminal_port(config_, config_.terminals[i]), bytes.data(), length);
            result = worst(result, s);
        }
        monitor(command, nullptr);
        return result;
    }

    TerminalStats& term = terminals_[rt];
    ++term.transactions;
    const Status sent = socket_.send_to(terminal_port(config_, rt), bytes.data(), length);
    if (sent != Status::kOk) {
        return sent;
    }
    if (!expect_reply) {
        monitor(command, nullptr);
        return Status::kOk;
    }

    const Status got = await_reply(command.transaction, reply);
    if (got != Status::kOk) {
        ++term.no_response;
        monitor(command, nullptr);
        return got;
    }
    term.responded = true;

    // The reply must come from the addressed terminal and be the right size
    // for the command, otherwise it is counted and not believed.
    bus::StatusWord sw;
    const Status sdec = bus::decode_status_word(reply.words[0], sw);
    std::uint8_t expected_words = 1;
    if (cw.is_mode_code()) {
        if (count_or_mode == bus::kModeCodeTransmitVectorWord ||
            count_or_mode == bus::kModeCodeTransmitLastCommand ||
            count_or_mode == bus::kModeCodeTransmitBitWord) {
            expected_words = 2;
        }
    } else if (transmit) {
        expected_words = static_cast<std::uint8_t>(1U + count_or_mode);
    }
    if (sdec != Status::kOk || sw.rt != rt || reply.word_count != expected_words) {
        ++term.malformed;
        monitor(command, &reply);
        return Status::kMalformed;
    }
    if (sw.message_error) {
        ++term.message_errors;
    }
    if (sw.busy) {
        ++term.busy;
    }
    if (transmit && !cw.is_mode_code()) {
        record_transmit_data(rt, sa, reply);
    }
    monitor(command, &reply);
    return Status::kOk;
}

Status BusController::await_reply(bus::Word transaction, bus::Frame& reply) noexcept {
    const osal::Nanoseconds deadline = osal::now() + config_.response_timeout;
    // Bounded: each iteration consumes one datagram or the remaining time.
    for (int attempt = 0; attempt < 64; ++attempt) {
        const osal::Nanoseconds remaining = deadline - osal::now();
        if (remaining <= 0) {
            return Status::kTimeout;
        }
        std::array<std::uint8_t, osal::kMaxDatagramBytes> bytes{};
        std::size_t length = 0;
        const Status r = socket_.receive(bytes.data(), bytes.size(), length, remaining, nullptr);
        if (r == Status::kTimeout) {
            return Status::kTimeout;
        }
        if (r != Status::kOk) {
            continue;  // an oversized or broken datagram is not our reply
        }
        bus::Frame f;
        if (bus::decode_frame(bytes.data(), length, f) != Status::kOk || f.type != bus::FrameType::kRtToBc ||
            f.word_count < 1) {
            continue;
        }
        if (f.transaction != transaction) {
            continue;  // a late reply to an earlier command
        }
        reply = f;
        return Status::kOk;
    }
    return Status::kTimeout;
}

void BusController::record_transmit_data(std::uint8_t rt, std::uint8_t sa, const bus::Frame& reply) noexcept {
    LatestData& d = latest_[rt][sa];
    const std::uint8_t count = static_cast<std::uint8_t>(reply.word_count - 1U);
    for (std::size_t i = 0; i < bus::kMaxDataWords; ++i) {
        d.words[i] = i < count ? reply.words[1 + i] : bus::Word{0};
    }
    d.count = count;
    d.frame = stats_.frames;
    d.valid = true;
}

void BusController::monitor(const bus::Frame& command, const bus::Frame* reply) noexcept {
    if (!config_.monitor_enabled) {
        return;
    }
    // One record per transaction: the command words as sent, then the reply
    // words (status first) if there was one.
    bus::Frame record;
    record.type = bus::FrameType::kMonitor;
    record.flags = command.flags;
    record.transaction = command.transaction;
    std::size_t n = 0;
    for (std::size_t i = 0; i < command.word_count && n < bus::kMaxFrameWords; ++i) {
        record.words[n++] = command.words[i];
    }
    if (reply != nullptr) {
        for (std::size_t i = 0; i < reply->word_count && n < bus::kMaxFrameWords; ++i) {
            record.words[n++] = reply->words[i];
        }
    }
    record.word_count = static_cast<std::uint8_t>(n);

    std::array<std::uint8_t, bus::kMaxFrameBytes> bytes{};
    std::size_t length = 0;
    if (bus::encode_frame(record, bytes.data(), bytes.size(), length) != Status::kOk ||
        socket_.send_to(monitor_port(config_), bytes.data(), length) != Status::kOk) {
        ++stats_.monitor_failures;
    }
}

}  // namespace fmofp::bc
