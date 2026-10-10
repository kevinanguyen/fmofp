// clang-format off
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
// clang-format on

#include <fmofp/osal/udp.hpp>

namespace fmofp::osal {

namespace {

sockaddr_in loopback(std::uint16_t port) noexcept {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return addr;
}

INT to_poll_ms(Nanoseconds timeout) noexcept {
    if (timeout <= 0) {
        return 0;
    }
    const Nanoseconds ms = (timeout + kNsPerMs - 1) / kNsPerMs;
    return ms > 60 * 1000 ? 60 * 1000 : static_cast<INT>(ms);
}

bool g_started = false;

}  // namespace

Status init() noexcept {
    if (g_started) {
        return Status::kOk;
    }
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
        return Status::kIoError;
    }
    g_started = true;
    return Status::kOk;
}

UdpSocket::~UdpSocket() { close(); }

Status UdpSocket::open(std::uint16_t port) noexcept {
    close();
    if (!g_started) {
        return Status::kNotReady;
    }
    const SOCKET s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) {
        return Status::kIoError;
    }
    const sockaddr_in addr = loopback(port);
    if (::bind(s, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) != 0) {
        ::closesocket(s);
        return Status::kIoError;
    }
    handle_ = static_cast<std::int64_t>(s);
    return Status::kOk;
}

void UdpSocket::close() noexcept {
    if (handle_ >= 0) {
        ::closesocket(static_cast<SOCKET>(handle_));
        handle_ = -1;
    }
}

Status UdpSocket::send_to(std::uint16_t port, const std::uint8_t* data, std::size_t length) noexcept {
    if (!is_open() || data == nullptr || length == 0 || length > kMaxDatagramBytes) {
        return Status::kInvalidArgument;
    }
    const sockaddr_in addr = loopback(port);
    const int sent = ::sendto(static_cast<SOCKET>(handle_), reinterpret_cast<const char*>(data),
                              static_cast<int>(length), 0, reinterpret_cast<const sockaddr*>(&addr),
                              sizeof addr);
    return sent == static_cast<int>(length) ? Status::kOk : Status::kIoError;
}

Status UdpSocket::receive(std::uint8_t* buffer, std::size_t capacity, std::size_t& length,
                          Nanoseconds timeout, std::uint16_t* from_port) noexcept {
    length = 0;
    if (!is_open() || buffer == nullptr || capacity == 0) {
        return Status::kInvalidArgument;
    }
    WSAPOLLFD pfd{};
    pfd.fd = static_cast<SOCKET>(handle_);
    pfd.events = POLLRDNORM;
    const int ready = ::WSAPoll(&pfd, 1, to_poll_ms(timeout));
    if (ready == 0) {
        return Status::kTimeout;
    }
    if (ready < 0) {
        return Status::kIoError;
    }

    sockaddr_in from{};
    int from_len = sizeof from;
    const int got = ::recvfrom(static_cast<SOCKET>(handle_), reinterpret_cast<char*>(buffer),
                               static_cast<int>(capacity), 0, reinterpret_cast<sockaddr*>(&from),
                               &from_len);
    if (got == SOCKET_ERROR) {
        // Winsock reports an oversized datagram as an error after discarding
        // the excess, which is the drop-and-report behaviour wanted here.
        return WSAGetLastError() == WSAEMSGSIZE ? Status::kBadLength : Status::kIoError;
    }
    length = static_cast<std::size_t>(got);
    if (from_port != nullptr) {
        *from_port = ntohs(from.sin_port);
    }
    return Status::kOk;
}

}  // namespace fmofp::osal
