#ifndef FMOFP_RT_RT_PROCESS_HPP
#define FMOFP_RT_RT_PROCESS_HPP

#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/osal/clock.hpp>
#include <fmofp/osal/udp.hpp>

// The socket end of a terminal: receives the bus controller's datagrams,
// hands them to the RemoteTerminal, and sends back whatever it answers.
// Every partition process owns one of these and calls poll() in its loop;
// the partition's own work happens when terminal().take_sync() fires.

namespace fmofp::rt {

struct RtProcessConfig {
    std::uint8_t rt{0};
    std::uint16_t port_base{51000};
    std::uint8_t bc_port_offset{32};
};

struct RtProcessStats {
    std::uint32_t datagrams{0};
    std::uint32_t replies{0};
    std::uint32_t bad_frames{0};  // not a decodable BC-to-RT frame
    std::uint32_t send_failures{0};
};

class RtProcess {
public:
    [[nodiscard]] Status init(const RtProcessConfig& cfg) noexcept;
    void shutdown() noexcept;

    // Waits up to `timeout` for one datagram and handles it. kTimeout when
    // nothing arrived; kOk when a datagram was processed, valid or not.
    [[nodiscard]] Status poll(osal::Nanoseconds timeout) noexcept;

    [[nodiscard]] bus::RemoteTerminal& terminal() noexcept { return terminal_; }
    [[nodiscard]] const bus::RemoteTerminal& terminal() const noexcept { return terminal_; }
    [[nodiscard]] const RtProcessStats& stats() const noexcept { return stats_; }
    [[nodiscard]] const RtProcessConfig& config() const noexcept { return config_; }

private:
    RtProcessConfig config_{};
    osal::UdpSocket socket_{};
    bus::RemoteTerminal terminal_{};
    RtProcessStats stats_{};
};

}  // namespace fmofp::rt

#endif
