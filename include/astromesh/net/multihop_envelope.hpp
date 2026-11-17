#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace astromesh::net {

/**
 * @brief Multihop routing envelope header flags.
 */
namespace MultihopFlags {
    constexpr uint8_t NONE = 0x00;
    constexpr uint8_t CUSTODY_ACK_REQUESTED = 0x01;  ///< Request hop-level custody ACK
    constexpr uint8_t E2E_ACK_REQUESTED     = 0x02;  ///< Request end-to-end delivery ACK
    constexpr uint8_t IS_CUSTODY_ACK        = 0x04;  ///< Frame is a hop-level custody ACK
    constexpr uint8_t IS_E2E_ACK            = 0x08;  ///< Frame is an end-to-end delivery ACK
    constexpr uint8_t VALID_FLAGS_MASK      = 0x0F;
}

/**
 * @brief Inner network envelope enabling bounded multi-hop mesh routing,
 * path-trace loop prevention, and immutable end-to-end message identity.
 * 
 * Wire format (40 bytes Little-Endian + Application Payload):
 *  - [0..3]:   MAGIC 'MHTN' (0x4E54484D)
 *  - [4]:      Version (1)
 *  - [5]:      Flags (MultihopFlags)
 *  - [6..7]:   Origin Rover ID (uint16_t, > 0)
 *  - [8..9]:   Final Target ID (uint16_t, 0 = broadcast)
 *  - [10]:     Hop Count (uint8_t, 0 <= hop_count <= max_hops)
 *  - [11]:     Max Hops (uint8_t, 1 <= max_hops <= 8)
 *  - [12..19]: Message ID (uint64_t immutable)
 *  - [20]:     Path Length (uint8_t, 1 <= path_length <= 8)
 *  - [21..23]: Reserved (3 bytes, zero padding)
 *  - [24..39]: Visited Nodes (8 * uint16_t = 16 bytes)
 *  - [40..N]:  Inner Application Payload
 */
struct MultihopEnvelope {
    static constexpr uint32_t MAGIC = 0x4E54484D; // 'MHTN' in Little-Endian
    static constexpr uint8_t CURRENT_VERSION = 1;
    static constexpr size_t HEADER_SIZE = 40;
    static constexpr uint8_t MAX_HOPS_LIMIT = 8;
    static constexpr size_t MAX_VISITED_NODES = 8;

    uint8_t version{CURRENT_VERSION};
    uint8_t flags{MultihopFlags::NONE};
    uint16_t origin_rover_id{0};
    uint16_t final_target_id{0};
    uint8_t hop_count{0};
    uint8_t max_hops{5};
    uint64_t message_id{0};
    uint8_t path_length{0};
    std::array<uint8_t, 3> reserved{0, 0, 0};
    std::array<uint16_t, MAX_VISITED_NODES> visited_nodes{0};

    std::vector<uint8_t> payload;

    /// Checks if a rover ID is in the visited nodes path.
    bool has_visited(uint16_t rover_id) const noexcept;

    /// Checks if adding current_rover_id would create a routing loop.
    bool is_loop_detected(uint16_t current_rover_id) const noexcept;

    /// Appends a relay rover ID to visited nodes. Returns false if full or loop.
    bool append_visited(uint16_t relay_rover_id) noexcept;

    /// Checks if the packet has reached or exceeded its hop limit.
    bool is_expired() const noexcept;

    /// Validates protocol invariants.
    bool validate() const noexcept;

    /// Serializes envelope header and payload to byte vector.
    std::vector<uint8_t> serialize() const;

    /// Deserializes byte buffer into MultihopEnvelope. Returns nullopt on validation failure.
    static std::optional<MultihopEnvelope> deserialize(const uint8_t* data, size_t size);
};

} // namespace astromesh::net
