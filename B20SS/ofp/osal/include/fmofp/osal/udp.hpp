#ifndef FMOFP_OSAL_UDP_HPP
#define FMOFP_OSAL_UDP_HPP

#include <cstddef>
#include <cstdint>

#include <fmofp/bus/status.hpp>
#include <fmofp/osal/clock.hpp>

// UDP datagrams on 127.0.0.1, the simulated wire of ICD README section 7.
// One datagram is one bus transaction, so nothing here reassembles anything.

namespace fmofp::osal {

// Largest datagram the bus ever needs: the 4-word transport header plus a
// command word, 32 data words and a status word is 38 words. Rounded up.
constexpr std::size_t kMaxDatagramBytes = 128;

// Must be called once per process before any socket is opened (Winsock
// start-up; a no-op elsewhere). Calling it again is harmless.
[[nodiscard]] Status init() noexcept;

class UdpSocket {
public:
    UdpSocket() noexcept = default;
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Binds to 127.0.0.1:port for receiving; the same socket sends.
    // kIoError if the port is taken or the OS refuses.
    [[nodiscard]] Status open(std::uint16_t port) noexcept;
    void close() noexcept;
    [[nodiscard]] bool is_open() const noexcept { return handle_ >= 0; }

    [[nodiscard]] Status send_to(std::uint16_t port, const std::uint8_t* data,
                                 std::size_t length) noexcept;

    // Waits up to `timeout` for one datagram. kTimeout if none arrives,
    // kBadLength if one arrives that does not fit `capacity` (it is dropped).
    // `from_port` may be null.
    [[nodiscard]] Status receive(std::uint8_t* buffer, std::size_t capacity, std::size_t& length,
                                 Nanoseconds timeout, std::uint16_t* from_port) noexcept;

private:
    std::int64_t handle_{-1};
};

}  // namespace fmofp::osal

#endif
