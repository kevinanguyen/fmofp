#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

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

int to_poll_ms(Nanoseconds timeout) noexcept {
    if (timeout <= 0) {
        return 0;
    }
    const Nanoseconds ms = (timeout + kNsPerMs - 1) / kNsPerMs;
    return ms > 60 * 1000 ? 60 * 1000 : static_cast<int>(ms);
}

}  // namespace

Status init() noexcept { return Status::kOk; }

UdpSocket::~UdpSocket() { close(); }

Status UdpSocket::open(std::uint16_t port) noexcept {
    close();
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return Status::kIoError;
    }
    const sockaddr_in addr = loopback(port);
    if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) != 0) {
        ::close(fd);
        return Status::kIoError;
    }
    handle_ = fd;
    return Status::kOk;
}

void UdpSocket::close() noexcept {
    if (handle_ >= 0) {
        ::close(static_cast<int>(handle_));
        handle_ = -1;
    }
}

Status UdpSocket::send_to(std::uint16_t port, const std::uint8_t* data, std::size_t length) noexcept {
    if (!is_open() || data == nullptr || length == 0 || length > kMaxDatagramBytes) {
        return Status::kInvalidArgument;
    }
    const sockaddr_in addr = loopback(port);
    const ssize_t sent = ::sendto(static_cast<int>(handle_), data, length, 0,
                                  reinterpret_cast<const sockaddr*>(&addr), sizeof addr);
    return sent == static_cast<ssize_t>(length) ? Status::kOk : Status::kIoError;
}

Status UdpSocket::receive(std::uint8_t* buffer, std::size_t capacity, std::size_t& length,
                          Nanoseconds timeout, std::uint16_t* from_port) noexcept {
    length = 0;
    if (!is_open() || buffer == nullptr || capacity == 0) {
        return Status::kInvalidArgument;
    }
    pollfd pfd{};
    pfd.fd = static_cast<int>(handle_);
    pfd.events = POLLIN;
    const int ready = ::poll(&pfd, 1, to_poll_ms(timeout));
    if (ready == 0) {
        return Status::kTimeout;
    }
    if (ready < 0) {
        return errno == EINTR ? Status::kTimeout : Status::kIoError;
    }

    sockaddr_in from{};
    socklen_t from_len = sizeof from;
    // MSG_TRUNC reports the full datagram size even when it was cut to fit,
    // which is how an oversized datagram is detected and dropped.
    const ssize_t got = ::recvfrom(static_cast<int>(handle_), buffer, capacity, MSG_TRUNC,
                                   reinterpret_cast<sockaddr*>(&from), &from_len);
    if (got < 0) {
        return Status::kIoError;
    }
    if (static_cast<std::size_t>(got) > capacity) {
        return Status::kBadLength;
    }
    length = static_cast<std::size_t>(got);
    if (from_port != nullptr) {
        *from_port = ntohs(from.sin_port);
    }
    return Status::kOk;
}

}  // namespace fmofp::osal
