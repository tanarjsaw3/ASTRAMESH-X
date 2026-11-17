#include "astromesh/optimization/boundary_packet.hpp"

#include <cmath>
#include <cstring>
#include <sstream>

namespace astromesh::optimization {

namespace {

inline void write_u16_le(uint8_t* dst, uint16_t v) noexcept {
    dst[0] = static_cast<uint8_t>(v & 0xFF);
    dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void write_u32_le(uint8_t* dst, uint32_t v) noexcept {
    dst[0] = static_cast<uint8_t>(v & 0xFF);
    dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline void write_u64_le(uint8_t* dst, uint64_t v) noexcept {
    for (size_t i = 0; i < 8; ++i) {
        dst[i] = static_cast<uint8_t>((v >> (i * 8)) & 0xFF);
    }
}

inline void write_f64_le(uint8_t* dst, double d) noexcept {
    uint64_t raw = 0;
    std::memcpy(&raw, &d, sizeof(double));
    write_u64_le(dst, raw);
}

inline uint16_t read_u16_le(const uint8_t* src) noexcept {
    return static_cast<uint16_t>(src[0]) |
          (static_cast<uint16_t>(src[1]) << 8);
}

inline uint32_t read_u32_le(const uint8_t* src) noexcept {
    return  static_cast<uint32_t>(src[0])        |
           (static_cast<uint32_t>(src[1]) << 8)  |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
}

inline uint64_t read_u64_le(const uint8_t* src) noexcept {
    uint64_t v = 0;
    for (size_t i = 0; i < 8; ++i) {
        v |= (static_cast<uint64_t>(src[i]) << (i * 8));
    }
    return v;
}

inline double read_f64_le(const uint8_t* src) noexcept {
    uint64_t raw = read_u64_le(src);
    double d = 0.0;
    std::memcpy(&d, &raw, sizeof(double));
    return d;
}

} // namespace

bool BoundaryStatePacket::operator==(const BoundaryStatePacket& other) const noexcept {
    constexpr double EPS = 1e-9;
    return protocol_version == other.protocol_version &&
           sender_rover_id == other.sender_rover_id &&
           target_rover_id == other.target_rover_id &&
           node_local_index == other.node_local_index &&
           sequence_number == other.sequence_number &&
           timestamp_ns == other.timestamp_ns &&
           std::abs(x - other.x) < EPS &&
           std::abs(y - other.y) < EPS &&
           std::abs(graph::normalize_angle(theta - other.theta)) < EPS &&
           std::abs(vx - other.vx) < EPS &&
           std::abs(vy - other.vy) < EPS &&
           std::abs(omega - other.omega) < EPS &&
           checksum == other.checksum;
}

uint64_t compute_packet_checksum(const uint8_t* buffer, size_t length) noexcept {
    // 64-bit FNV-1a hash followed by SplitMix64 mixing
    uint64_t hash = 14695981039346656037ULL;
    for (size_t i = 0; i < length; ++i) {
        hash ^= static_cast<uint64_t>(buffer[i]);
        hash *= 1099511628211ULL;
    }
    hash ^= hash >> 30;
    hash *= 0xbf58476d1ce4e5b9ULL;
    hash ^= hash >> 27;
    hash *= 0x94d049bb133111ebULL;
    hash ^= hash >> 31;
    return hash;
}

std::vector<uint8_t> serialize_boundary_packet(const BoundaryStatePacket& packet) {
    std::vector<uint8_t> buffer(BOUNDARY_PACKET_WIRE_SIZE, 0);

    write_u16_le(&buffer[0],  packet.protocol_version);
    write_u16_le(&buffer[2],  packet.sender_rover_id);
    write_u16_le(&buffer[4],  packet.target_rover_id);
    write_u16_le(&buffer[6],  packet.reserved);
    write_u32_le(&buffer[8],  packet.node_local_index);
    write_u32_le(&buffer[12], packet.sequence_number);
    write_u64_le(&buffer[16], packet.timestamp_ns);
    write_f64_le(&buffer[24], packet.x);
    write_f64_le(&buffer[32], packet.y);
    write_f64_le(&buffer[40], graph::normalize_angle(packet.theta));
    write_f64_le(&buffer[48], packet.vx);
    write_f64_le(&buffer[56], packet.vy);
    write_f64_le(&buffer[64], packet.omega);

    // Compute checksum over bytes 0..71
    uint64_t chk = compute_packet_checksum(buffer.data(), 72);
    write_u64_le(&buffer[72], chk);

    return buffer;
}

PacketValidationResult deserialize_boundary_packet(const uint8_t* data, size_t length) {
    PacketValidationResult res;
    if (data == nullptr) {
        res.valid = false;
        res.error_message = "Null buffer pointer provided";
        return res;
    }

    if (length != BOUNDARY_PACKET_WIRE_SIZE) {
        res.valid = false;
        std::ostringstream oss;
        oss << "Invalid packet length " << length << " bytes (expected exactly "
            << BOUNDARY_PACKET_WIRE_SIZE << " bytes)";
        res.error_message = oss.str();
        return res;
    }

    const uint16_t version = read_u16_le(&data[0]);
    if (version != BOUNDARY_PROTOCOL_VERSION) {
        res.valid = false;
        std::ostringstream oss;
        oss << "Unsupported protocol version " << version << " (expected "
            << BOUNDARY_PROTOCOL_VERSION << ")";
        res.error_message = oss.str();
        return res;
    }

    const uint64_t wire_checksum = read_u64_le(&data[72]);
    const uint64_t computed_checksum = compute_packet_checksum(data, 72);
    if (wire_checksum != computed_checksum) {
        res.valid = false;
        std::ostringstream oss;
        oss << "Checksum mismatch: wire 0x" << std::hex << wire_checksum
            << " != computed 0x" << computed_checksum;
        res.error_message = oss.str();
        return res;
    }

    BoundaryStatePacket pkt;
    pkt.protocol_version = version;
    pkt.sender_rover_id = read_u16_le(&data[2]);
    pkt.target_rover_id = read_u16_le(&data[4]);
    pkt.reserved = read_u16_le(&data[6]);
    pkt.node_local_index = read_u32_le(&data[8]);
    pkt.sequence_number = read_u32_le(&data[12]);
    pkt.timestamp_ns = read_u64_le(&data[16]);
    pkt.x = read_f64_le(&data[24]);
    pkt.y = read_f64_le(&data[32]);
    pkt.theta = graph::normalize_angle(read_f64_le(&data[40]));
    pkt.vx = read_f64_le(&data[48]);
    pkt.vy = read_f64_le(&data[56]);
    pkt.omega = read_f64_le(&data[64]);
    pkt.checksum = wire_checksum;

    if (!pkt.is_valid()) {
        res.valid = false;
        res.error_message = "Packet contains non-positive identifiers or non-finite pose/velocity values";
        return res;
    }

    res.valid = true;
    res.packet = pkt;
    return res;
}

} // namespace astromesh::optimization
