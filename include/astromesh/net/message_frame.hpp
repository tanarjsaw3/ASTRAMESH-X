#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::net {

/**
 * @brief Formal protocol message types for inter-rover communication.
 */
enum class MessageType : uint16_t {
    Heartbeat = 1,
    LandmarkSync = 2,
    CandidateProposal = 3,
    CandidateResponse = 4,
    BoundaryState = 5,
    Ack = 6,
    DataPayload = 7,
    DisconnectNotice = 8,
    RoutingAdvertisement = 9,
    MultihopForward = 10
};

inline const char* to_string(MessageType t) noexcept {
    switch (t) {
        case MessageType::Heartbeat: return "Heartbeat";
        case MessageType::LandmarkSync: return "LandmarkSync";
        case MessageType::CandidateProposal: return "CandidateProposal";
        case MessageType::CandidateResponse: return "CandidateResponse";
        case MessageType::BoundaryState: return "BoundaryState";
        case MessageType::Ack: return "Ack";
        case MessageType::DataPayload: return "DataPayload";
        case MessageType::DisconnectNotice: return "DisconnectNotice";
        case MessageType::RoutingAdvertisement: return "RoutingAdvertisement";
        case MessageType::MultihopForward: return "MultihopForward";
        default: return "Unknown";
    }
}

/**
 * @brief Versioned, checksum-protected binary message frame for peer-to-peer rover communication.
 * Wire Header: 36 bytes (Little-Endian).
 */
struct MessageFrame {
    static constexpr uint32_t MAGIC = 0x58545341; // 'ASTX' in little-endian
    static constexpr uint16_t CURRENT_SCHEMA_VERSION = 1;
    static constexpr uint16_t MIN_SUPPORTED_SCHEMA_VERSION = 1;
    static constexpr uint16_t MAX_SUPPORTED_SCHEMA_VERSION = 1;
    static constexpr size_t HEADER_SIZE = 36;
    static constexpr size_t MAX_PAYLOAD_SIZE = 2048;

    static constexpr bool is_supported_schema_version(uint16_t version) noexcept {
        return version >= MIN_SUPPORTED_SCHEMA_VERSION && version <= MAX_SUPPORTED_SCHEMA_VERSION;
    }

    uint16_t schema_version{CURRENT_SCHEMA_VERSION};
    MessageType type{MessageType::Heartbeat};
    uint16_t sender_rover_id{0};
    uint16_t target_rover_id{0}; // 0 = broadcast to all peers
    uint32_t sequence{0};
    uint64_t timestamp_ns{0};
    uint64_t checksum{0};
    std::vector<uint8_t> payload;

    /**
     * @brief Computes 64-bit SplitMix64/FNV-1a checksum over header fields and payload.
     */
    uint64_t compute_checksum() const noexcept;

    /**
     * @brief Verifies whether the frame's stored checksum matches its contents.
     */
    bool verify_checksum() const noexcept;

    /**
     * @brief Serializes frame into little-endian binary bytes.
     */
    std::vector<uint8_t> serialize() const;

    /**
     * @brief Deserializes and validates frame from raw binary bytes.
     * @return Deserialized frame, or nullopt if magic, length, or checksum fails.
     */
    static std::optional<MessageFrame> deserialize(const uint8_t* data, size_t size) noexcept;
};

} // namespace astromesh::net

