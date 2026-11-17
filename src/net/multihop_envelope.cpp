#include "astromesh/net/multihop_envelope.hpp"

#include <cstring>

namespace astromesh::net {

bool MultihopEnvelope::has_visited(uint16_t rover_id) const noexcept {
    if (rover_id == 0) return false;
    const size_t limit = (path_length <= MAX_VISITED_NODES) ? path_length : MAX_VISITED_NODES;
    for (size_t i = 0; i < limit; ++i) {
        if (visited_nodes[i] == rover_id) {
            return true;
        }
    }
    return false;
}

bool MultihopEnvelope::is_loop_detected(uint16_t current_rover_id) const noexcept {
    return has_visited(current_rover_id);
}

bool MultihopEnvelope::append_visited(uint16_t relay_rover_id) noexcept {
    if (relay_rover_id == 0) return false;
    if (path_length >= MAX_VISITED_NODES) return false;
    if (has_visited(relay_rover_id)) return false;

    visited_nodes[path_length++] = relay_rover_id;
    return true;
}

bool MultihopEnvelope::is_expired() const noexcept {
    return hop_count >= max_hops;
}

bool MultihopEnvelope::validate() const noexcept {
    if (version != CURRENT_VERSION) return false;
    if ((flags & ~MultihopFlags::VALID_FLAGS_MASK) != 0) return false;
    if (origin_rover_id == 0) return false;
    if (max_hops == 0 || max_hops > MAX_HOPS_LIMIT) return false;
    if (hop_count > max_hops) return false;
    if (path_length == 0 || path_length > MAX_VISITED_NODES) return false;
    if (path_length > static_cast<size_t>(hop_count) + 1) return false;
    if (visited_nodes[0] != origin_rover_id) return false;

    // Check no zeros or duplicates in visited path
    for (size_t i = 0; i < path_length; ++i) {
        if (visited_nodes[i] == 0) return false;
        for (size_t j = i + 1; j < path_length; ++j) {
            if (visited_nodes[i] == visited_nodes[j]) {
                return false; // Loop / duplicate entry detected
            }
        }
    }

    return true;
}

std::vector<uint8_t> MultihopEnvelope::serialize() const {
    std::vector<uint8_t> buf(HEADER_SIZE + payload.size());

    // [0..3]: MAGIC
    buf[0] = static_cast<uint8_t>(MAGIC & 0xFF);
    buf[1] = static_cast<uint8_t>((MAGIC >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>((MAGIC >> 16) & 0xFF);
    buf[3] = static_cast<uint8_t>((MAGIC >> 24) & 0xFF);

    // [4]: Version
    buf[4] = version;

    // [5]: Flags
    buf[5] = flags;

    // [6..7]: Origin Rover ID
    buf[6] = static_cast<uint8_t>(origin_rover_id & 0xFF);
    buf[7] = static_cast<uint8_t>((origin_rover_id >> 8) & 0xFF);

    // [8..9]: Final Target ID
    buf[8] = static_cast<uint8_t>(final_target_id & 0xFF);
    buf[9] = static_cast<uint8_t>((final_target_id >> 8) & 0xFF);

    // [10]: Hop Count
    buf[10] = hop_count;

    // [11]: Max Hops
    buf[11] = max_hops;

    // [12..19]: Message ID
    for (size_t i = 0; i < 8; ++i) {
        buf[12 + i] = static_cast<uint8_t>((message_id >> (8 * i)) & 0xFF);
    }

    // [20]: Path Length
    buf[20] = path_length;

    // [21..23]: Reserved
    buf[21] = 0;
    buf[22] = 0;
    buf[23] = 0;

    // [24..39]: Visited Nodes (8 * 2 = 16 bytes)
    for (size_t i = 0; i < MAX_VISITED_NODES; ++i) {
        uint16_t node_id = (i < path_length) ? visited_nodes[i] : 0;
        buf[24 + 2 * i]     = static_cast<uint8_t>(node_id & 0xFF);
        buf[24 + 2 * i + 1] = static_cast<uint8_t>((node_id >> 8) & 0xFF);
    }

    // [40..N]: Payload
    if (!payload.empty()) {
        std::memcpy(buf.data() + HEADER_SIZE, payload.data(), payload.size());
    }

    return buf;
}

std::optional<MultihopEnvelope> MultihopEnvelope::deserialize(const uint8_t* data, size_t size) {
    if (data == nullptr || size < HEADER_SIZE) {
        return std::nullopt;
    }

    // Check MAGIC
    uint32_t magic = static_cast<uint32_t>(data[0]) |
                    (static_cast<uint32_t>(data[1]) << 8) |
                    (static_cast<uint32_t>(data[2]) << 16) |
                    (static_cast<uint32_t>(data[3]) << 24);
    if (magic != MAGIC) {
        return std::nullopt;
    }

    MultihopEnvelope env;
    env.version = data[4];
    env.flags = data[5];

    env.origin_rover_id = static_cast<uint16_t>(data[6]) |
                         (static_cast<uint16_t>(data[7]) << 8);

    env.final_target_id = static_cast<uint16_t>(data[8]) |
                         (static_cast<uint16_t>(data[9]) << 8);

    env.hop_count = data[10];
    env.max_hops  = data[11];

    env.message_id = 0;
    for (size_t i = 0; i < 8; ++i) {
        env.message_id |= (static_cast<uint64_t>(data[12 + i]) << (8 * i));
    }

    env.path_length = data[20];
    env.reserved = {data[21], data[22], data[23]};

    env.visited_nodes.fill(0);
    for (size_t i = 0; i < MAX_VISITED_NODES; ++i) {
        env.visited_nodes[i] = static_cast<uint16_t>(data[24 + 2 * i]) |
                              (static_cast<uint16_t>(data[24 + 2 * i + 1]) << 8);
    }

    // Extract inner payload
    const size_t payload_size = size - HEADER_SIZE;
    if (payload_size > 0) {
        env.payload.resize(payload_size);
        std::memcpy(env.payload.data(), data + HEADER_SIZE, payload_size);
    }

    if (!env.validate()) {
        return std::nullopt;
    }

    return env;
}

} // namespace astromesh::net
