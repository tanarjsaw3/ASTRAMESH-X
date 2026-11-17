#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace astromesh::net {

/**
 * @brief Network endpoint address (IP and UDP port).
 */
struct Endpoint {
    std::string ip{"127.0.0.1"};
    uint16_t port{0};

    Endpoint() = default;
    Endpoint(std::string ip_, uint16_t port_) : ip(std::move(ip_)), port(port_) {}

    bool is_valid() const noexcept {
        return !ip.empty() && port > 0;
    }

    bool operator==(const Endpoint& o) const noexcept {
        return port == o.port && ip == o.ip;
    }

    bool operator!=(const Endpoint& o) const noexcept {
        return !(*this == o);
    }

    std::string to_string() const {
        return ip + ":" + std::to_string(port);
    }
};

/**
 * @brief Cross-platform RAII UDP socket wrapper supporting non-blocking I/O and timeouts.
 */
class UdpSocket {
public:
    UdpSocket();
    ~UdpSocket();

    // Non-copyable, movable
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    UdpSocket(UdpSocket&& other) noexcept;
    UdpSocket& operator=(UdpSocket&& other) noexcept;

    /**
     * @brief Creates and opens the underlying OS UDP socket.
     */
    bool open();

    /**
     * @brief Closes the socket.
     */
    void close();

    /**
     * @brief Binds the socket to a local port and IP.
     */
    bool bind(uint16_t port, const std::string& ip = "127.0.0.1");

    /**
     * @brief Sets non-blocking socket mode.
     */
    bool set_nonblocking(bool nonblocking);

    /**
     * @brief Sets receive timeout in milliseconds.
     */
    bool set_recv_timeout_ms(uint32_t timeout_ms);

    /**
     * @brief Sends datagram buffer to destination endpoint.
     * @return Number of bytes sent, or -1 on error.
     */
    int64_t send_to(const uint8_t* data, size_t size, const Endpoint& dest);

    /**
     * @brief Receives datagram into buffer.
     * @param buffer Pre-allocated memory.
     * @param buffer_size Capacity of buffer.
     * @param sender_out Populated with sender IP and port.
     * @return Number of bytes received, 0 on timeout/wouldblock, or -1 on fatal error.
     */
    int64_t recv_from(uint8_t* buffer, size_t buffer_size, Endpoint& sender_out);

    bool is_open() const noexcept;
    uint16_t get_bound_port() const noexcept { return m_bound_port; }

private:
    uintptr_t m_socket; // Socket descriptor (SOCKET on Windows, int on POSIX)
    uint16_t m_bound_port{0};
};

} // namespace astromesh::net
