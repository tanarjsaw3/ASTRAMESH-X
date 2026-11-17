#include "astromesh/net/udp_socket.hpp"

#include <cstring>
#include <iostream>
#include <mutex>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

using native_sock_t = SOCKET;
constexpr native_sock_t INVALID_NATIVE_SOCK = INVALID_SOCKET;

static void ensure_winsock_init() {
    static std::once_flag init_flag;
    std::call_once(init_flag, []() {
        WSADATA wsa_data;
        int res = WSAStartup(MAKEWORD(2, 2), &wsa_data);
        if (res != 0) {
            std::cerr << "[UdpSocket] WSAStartup failed: " << res << "\n";
        }
    });
}
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using native_sock_t = int;
constexpr native_sock_t INVALID_NATIVE_SOCK = -1;
static void ensure_winsock_init() {}
#endif

namespace astromesh::net {

UdpSocket::UdpSocket() : m_socket(static_cast<uintptr_t>(INVALID_NATIVE_SOCK)), m_bound_port(0) {
    ensure_winsock_init();
}

UdpSocket::~UdpSocket() {
    close();
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
    : m_socket(other.m_socket), m_bound_port(other.m_bound_port) {
    other.m_socket = static_cast<uintptr_t>(INVALID_NATIVE_SOCK);
    other.m_bound_port = 0;
}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
    if (this != &other) {
        close();
        m_socket = other.m_socket;
        m_bound_port = other.m_bound_port;
        other.m_socket = static_cast<uintptr_t>(INVALID_NATIVE_SOCK);
        other.m_bound_port = 0;
    }
    return *this;
}

bool UdpSocket::open() {
    if (is_open()) return true;

    native_sock_t sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_NATIVE_SOCK) {
        return false;
    }

#ifdef _WIN32
    // Enable SO_REUSEADDR
    BOOL reuse = TRUE;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));
#else
    int reuse = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#endif

    m_socket = static_cast<uintptr_t>(sock);
    return true;
}

void UdpSocket::close() {
    native_sock_t sock = static_cast<native_sock_t>(m_socket);
    if (sock != INVALID_NATIVE_SOCK) {
#ifdef _WIN32
        ::closesocket(sock);
#else
        ::close(sock);
#endif
        m_socket = static_cast<uintptr_t>(INVALID_NATIVE_SOCK);
        m_bound_port = 0;
    }
}

bool UdpSocket::bind(uint16_t port, const std::string& ip) {
    if (!is_open()) {
        if (!open()) return false;
    }

    native_sock_t sock = static_cast<native_sock_t>(m_socket);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (ip == "0.0.0.0" || ip.empty()) {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
            return false;
        }
    }

    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        return false;
    }

    // If port was 0, query bound port
    if (port == 0) {
#ifdef _WIN32
        int len = sizeof(addr);
#else
        socklen_t len = sizeof(addr);
#endif
        if (::getsockname(sock, reinterpret_cast<sockaddr*>(&addr), &len) == 0) {
            m_bound_port = ntohs(addr.sin_port);
        }
    } else {
        m_bound_port = port;
    }

    return true;
}

bool UdpSocket::set_nonblocking(bool nonblocking) {
    if (!is_open()) return false;
    native_sock_t sock = static_cast<native_sock_t>(m_socket);

#ifdef _WIN32
    u_long mode = nonblocking ? 1 : 0;
    return (::ioctlsocket(sock, FIONBIO, &mode) == 0);
#else
    int flags = ::fcntl(sock, F_GETFL, 0);
    if (flags < 0) return false;
    flags = nonblocking ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    return (::fcntl(sock, F_SETFL, flags) == 0);
#endif
}

bool UdpSocket::set_recv_timeout_ms(uint32_t timeout_ms) {
    if (!is_open()) return false;
    native_sock_t sock = static_cast<native_sock_t>(m_socket);

#ifdef _WIN32
    DWORD timeout = timeout_ms;
    return (::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) == 0);
#else
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    return (::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0);
#endif
}

int64_t UdpSocket::send_to(const uint8_t* data, size_t size, const Endpoint& dest) {
    if (!is_open() || !data || size == 0 || !dest.is_valid()) return -1;
    native_sock_t sock = static_cast<native_sock_t>(m_socket);

    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(dest.port);
    if (::inet_pton(AF_INET, dest.ip.c_str(), &dest_addr.sin_addr) <= 0) {
        return -1;
    }

#ifdef _WIN32
    int sent = ::sendto(sock, reinterpret_cast<const char*>(data), static_cast<int>(size), 0,
                        reinterpret_cast<sockaddr*>(&dest_addr), sizeof(dest_addr));
    return (sent == SOCKET_ERROR) ? -1 : static_cast<int64_t>(sent);
#else
    ssize_t sent = ::sendto(sock, data, size, 0,
                            reinterpret_cast<sockaddr*>(&dest_addr), sizeof(dest_addr));
    return static_cast<int64_t>(sent);
#endif
}

int64_t UdpSocket::recv_from(uint8_t* buffer, size_t buffer_size, Endpoint& sender_out) {
    if (!is_open() || !buffer || buffer_size == 0) return -1;
    native_sock_t sock = static_cast<native_sock_t>(m_socket);

    sockaddr_in sender_addr{};
#ifdef _WIN32
    int addr_len = sizeof(sender_addr);
    int bytes = ::recvfrom(sock, reinterpret_cast<char*>(buffer), static_cast<int>(buffer_size), 0,
                           reinterpret_cast<sockaddr*>(&sender_addr), &addr_len);
    if (bytes == SOCKET_ERROR) {
        int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK || err == WSAETIMEDOUT) {
            return 0; // Normal non-blocking timeout
        }
        return -1;
    }
#else
    socklen_t addr_len = sizeof(sender_addr);
    ssize_t bytes = ::recvfrom(sock, buffer, buffer_size, 0,
                               reinterpret_cast<sockaddr*>(&sender_addr), &addr_len);
    if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; // Normal timeout
        }
        return -1;
    }
#endif

    char ip_buf[INET_ADDRSTRLEN];
    if (::inet_ntop(AF_INET, &sender_addr.sin_addr, ip_buf, sizeof(ip_buf))) {
        sender_out.ip = ip_buf;
    } else {
        sender_out.ip = "127.0.0.1";
    }
    sender_out.port = ntohs(sender_addr.sin_port);

    return static_cast<int64_t>(bytes);
}

bool UdpSocket::is_open() const noexcept {
    return static_cast<native_sock_t>(m_socket) != INVALID_NATIVE_SOCK;
}

} // namespace astromesh::net

