#include <fmofp/rt/rt_process.hpp>

#include <array>
#include <cstddef>

#include <fmofp/bus/transport.hpp>

namespace fmofp::rt {

Status RtProcess::init(const RtProcessConfig& cfg) noexcept {
    shutdown();
    const Status t = terminal_.init(cfg.rt);
    if (t != Status::kOk) {
        return t;
    }
    config_ = cfg;
    stats_ = RtProcessStats{};
    const Status o = osal::init();
    if (o != Status::kOk) {
        return o;
    }
    return socket_.open(static_cast<std::uint16_t>(config_.port_base + config_.rt));
}

void RtProcess::shutdown() noexcept { socket_.close(); }

Status RtProcess::poll(osal::Nanoseconds timeout) noexcept {
    if (!socket_.is_open()) {
        return Status::kNotReady;
    }
    std::array<std::uint8_t, osal::kMaxDatagramBytes> bytes{};
    std::size_t length = 0;
    const Status r = socket_.receive(bytes.data(), bytes.size(), length, timeout, nullptr);
    if (r == Status::kTimeout) {
        return Status::kTimeout;
    }
    ++stats_.datagrams;
    if (r != Status::kOk) {
        ++stats_.bad_frames;
        return Status::kOk;
    }

    bus::Frame in;
    if (bus::decode_frame(bytes.data(), length, in) != Status::kOk) {
        ++stats_.bad_frames;
        return Status::kOk;
    }
    bus::Frame reply;
    bool reply_needed = false;
    if (terminal_.handle(in, reply, reply_needed) != Status::kOk) {
        ++stats_.bad_frames;
        return Status::kOk;
    }
    if (!reply_needed) {
        return Status::kOk;
    }

    std::array<std::uint8_t, bus::kMaxFrameBytes> out{};
    std::size_t out_length = 0;
    if (bus::encode_frame(reply, out.data(), out.size(), out_length) != Status::kOk ||
        socket_.send_to(static_cast<std::uint16_t>(config_.port_base + config_.bc_port_offset), out.data(),
                        out_length) != Status::kOk) {
        ++stats_.send_failures;
        return Status::kOk;
    }
    ++stats_.replies;
    return Status::kOk;
}

}  // namespace fmofp::rt
