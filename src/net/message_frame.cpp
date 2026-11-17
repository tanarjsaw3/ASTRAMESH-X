#include "astromesh/net/message_frame.hpp"

#include <cstring>

namespace astromesh::net {

static inline uint64_t splitmix64(uint64_t x) noexcept {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

static inline void hash_update(uint64_t& h, uint64_t val) noexcept {
    h ^= splitmix64(val);
    h *= 0x100000001b3ULL; // FNV prime
}

uint64_t MessageFrame::compute_checksum() const noexcept {
    uint64_t h = 0xcbf29ce484222325ULL; // FNV offset basis
    hash_update(h, MAGIC);
    hash_update(h, schema_version);
    hash_update(h, static_cast<uint16_t>(type));
    hash_update(h, (static_cast<uint64_t>(sender_rover_id) << 16) | target_rover_id);
    hash_update(h, sequence);
    hash_update(h, timestamp_ns);
    hash_update(h, payload.size());

    // Mix payload bytes in 8-byte chunks
    size_t i = 0;
    while (i + 8 <= payload.size()) {
        uint64_t chunk = 0;
        std::memcpy(&chunk, &payload[i], 8);
        hash_update(h, chunk);
        i += 8;
    }
    if (i < payload.size()) {
        uint64_t tail = 0;
        std::memcpy(&tail, &payload[i], payload.size() - i);
        hash_update(h, tail);
    }

    return splitmix64(h);
}

bool MessageFrame::verify_checksum() const noexcept {
    return checksum == compute_checksum();
}

std::vector<uint8_t> MessageFrame::serialize() const {
    const uint32_t payload_len = static_cast<uint32_t>(payload.size());
    std::vector<uint8_t> buf(HEADER_SIZE + payload_len);

    // Compute checksum over fields
    const uint64_t computed_ck = compute_checksum();

    // Little-endian serialization helper
    auto write_u16 = [&](size_t offset, uint16_t v) {
        buf[offset] = static_cast<uint8_t>(v & 0xFF);
        buf[offset + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };

    auto write_u32 = [&](size_t offset, uint32_t v) {
        buf[offset] = static_cast<uint8_t>(v & 0xFF);
        buf[offset + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
        buf[offset + 2] = static_cast<uint8_t>((v >> 16) & 0xFF);
        buf[offset + 3] = static_cast<uint8_t>((v >> 24) & 0xFF);
    };

    auto write_u64 = [&](size_t offset, uint64_t v) {
        for (size_t b = 0; b < 8; ++b) {
            buf[offset + b] = static_cast<uint8_t>((v >> (b * 8)) & 0xFF);
        }
    };

    // Header layout (36 bytes):
    // [0..3]:   MAGIC (0x58545341)
    // [4..5]:   schema_version
    // [6..7]:   message_type
    // [8..9]:   sender_rover_id
    // [10..11]: target_rover_id
    // [12..15]: sequence
    // [16..23]: timestamp_ns
    // [24..27]: payload_length
    // [28..35]: checksum
    write_u32(0, MAGIC);
    write_u16(4, schema_version);
    write_u16(6, static_cast<uint16_t>(type));
    write_u16(8, sender_rover_id);
    write_u16(10, target_rover_id);
    write_u32(12, sequence);
    write_u64(16, timestamp_ns);
    write_u32(24, payload_len);
    write_u64(28, computed_ck);

    if (payload_len > 0) {
        std::memcpy(&buf[HEADER_SIZE], payload.data(), payload_len);
    }

    return buf;
}

std::optional<MessageFrame> MessageFrame::deserialize(const uint8_t* data, size_t size) noexcept {
    if (!data || size < HEADER_SIZE) return std::nullopt;

    auto read_u16 = [&](size_t offset) -> uint16_t {
        return static_cast<uint16_t>(data[offset]) |
               (static_cast<uint16_t>(data[offset + 1]) << 8);
    };

    auto read_u32 = [&](size_t offset) -> uint32_t {
        return static_cast<uint32_t>(data[offset]) |
               (static_cast<uint32_t>(data[offset + 1]) << 8) |
               (static_cast<uint32_t>(data[offset + 2]) << 16) |
               (static_cast<uint32_t>(data[offset + 3]) << 24);
    };

    auto read_u64 = [&](size_t offset) -> uint64_t {
        uint64_t v = 0;
        for (size_t b = 0; b < 8; ++b) {
            v |= (static_cast<uint64_t>(data[offset + b]) << (b * 8));
        }
        return v;
    };

    const uint32_t magic = read_u32(0);
    if (magic != MAGIC) return std::nullopt;

    const uint16_t version = read_u16(4);
    if (!is_supported_schema_version(version)) return std::nullopt;

    MessageFrame frame;
    frame.schema_version = version;
    frame.type = static_cast<MessageType>(read_u16(6));
    frame.sender_rover_id = read_u16(8);
    frame.target_rover_id = read_u16(10);
    frame.sequence = read_u32(12);
    frame.timestamp_ns = read_u64(16);
    const uint32_t payload_len = read_u32(24);
    frame.checksum = read_u64(28);

    if (payload_len > MAX_PAYLOAD_SIZE) return std::nullopt;
    if (size != HEADER_SIZE + payload_len) return std::nullopt;

    if (payload_len > 0) {
        frame.payload.resize(payload_len);
        std::memcpy(frame.payload.data(), &data[HEADER_SIZE], payload_len);
    }

    // Verify cryptographic integrity
    if (!frame.verify_checksum()) {
        return std::nullopt;
    }

    return frame;
}

} // namespace astromesh::net

