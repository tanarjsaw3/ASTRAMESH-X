#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::optimization {

constexpr uint16_t BOUNDARY_PROTOCOL_VERSION = 1;
constexpr size_t BOUNDARY_PACKET_WIRE_SIZE = 80;

/**
 * @brief Canonical boundary state packet exchanged between neighboring rovers.
 *
 * Wire Layout (80 bytes, explicit little-endian):
 *   [0..1]   protocol_version   (uint16_t, must equal 1)
 *   [2..3]   sender_rover_id    (uint16_t, > 0)
 *   [4..5]   target_rover_id    (uint16_t, > 0)
 *   [6..7]   reserved           (uint16_t, 0)
 *   [8..11]  node_local_index   (uint32_t, > 0)
 *   [12..15] sequence_number    (uint32_t)
 *   [16..23] timestamp_ns       (uint64_t)
 *   [24..31] x                  (double, IEEE-754)
 *   [32..39] y                  (double, IEEE-754)
 *   [40..47] theta              (double, IEEE-754)
 *   [48..55] vx                 (double, IEEE-754)
 *   [56..63] vy                 (double, IEEE-754)
 *   [64..71] omega              (double, IEEE-754)
 *   [72..79] checksum           (uint64_t, FNV-1a + SplitMix64 over bytes 0..71)
 */
struct BoundaryStatePacket {
    uint16_t protocol_version{BOUNDARY_PROTOCOL_VERSION};
    uint16_t sender_rover_id{0};
    uint16_t target_rover_id{0};
    uint16_t reserved{0};
    uint32_t node_local_index{0};
    uint32_t sequence_number{0};
    uint64_t timestamp_ns{0};
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};
    uint64_t checksum{0};

    graph::NodeId node_id() const noexcept {
        return graph::NodeId(sender_rover_id, node_local_index);
    }

    geometry::SE2 pose() const noexcept {
        return geometry::SE2(x, y, theta);
    }

    geometry::Twist2D velocity() const noexcept {
        return geometry::Twist2D(vx, vy, omega);
    }

    bool is_valid() const noexcept {
        return protocol_version == BOUNDARY_PROTOCOL_VERSION &&
               sender_rover_id > 0 && target_rover_id > 0 &&
               std::isfinite(x) && std::isfinite(y) && std::isfinite(theta) &&
               std::isfinite(vx) && std::isfinite(vy) && std::isfinite(omega);
    }

    bool operator==(const BoundaryStatePacket& other) const noexcept;
    bool operator!=(const BoundaryStatePacket& other) const noexcept {
        return !(*this == other);
    }
};

/**
 * @brief Computes 64-bit deterministic non-cryptographic checksum (FNV-1a followed by SplitMix64 output mixer) over a byte buffer.
 */
uint64_t compute_packet_checksum(const uint8_t* buffer, size_t length) noexcept;

/**
 * @brief Evaluates whether an incoming sequence number is strictly newer than current,
 *        handling RFC 1982 modular serial number wraparound.
 */
inline bool is_newer_sequence(uint32_t incoming_seq, uint32_t current_seq) noexcept {
    return static_cast<int32_t>(incoming_seq - current_seq) > 0;
}

/**
 * @brief Serializes a BoundaryStatePacket into an explicit portable 80-byte little-endian buffer.
 */
std::vector<uint8_t> serialize_boundary_packet(const BoundaryStatePacket& packet);

/**
 * @brief Structured outcome of boundary packet deserialization.
 */
struct PacketValidationResult {
    bool valid{false};
    std::string error_message;
    std::optional<BoundaryStatePacket> packet{std::nullopt};
};

/**
 * @brief Deserializes and strictly validates an 80-byte boundary state packet.
 */
PacketValidationResult deserialize_boundary_packet(const uint8_t* data, size_t length);

} // namespace astromesh::optimization
