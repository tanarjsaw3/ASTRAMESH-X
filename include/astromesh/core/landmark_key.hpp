#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <tuple>
#include <iosfwd>

namespace astromesh::core {

/**
 * @brief Represents a deterministic, versioned landmark key in the AstroMesh-X architecture.
 *
 * Each landmark key uniquely identifies a map landmark within the swarm:
 * - schema_version: Version of the landmark extractor/descriptor schema (0 is reserved as invalid).
 * - rover_id: Originating rover that first indexed the landmark.
 * - local_index: Local landmark sequence index assigned by the observing rover.
 * - descriptor_hash: Deterministic 64-bit hash of the visual/geometric landmark descriptor.
 */
class LandmarkKey {
public:
    // Reserved invalid schema version constant
    static constexpr uint16_t INVALID_SCHEMA = 0;

    /// Default constructor creates an invalid landmark key.
    constexpr LandmarkKey() noexcept
        : schema_version_(INVALID_SCHEMA), rover_id_(0), local_index_(0), descriptor_hash_(0) {}

    /**
     * @brief Construct a LandmarkKey with explicit fields.
     * @throws std::invalid_argument if schema_version is INVALID_SCHEMA (0).
     */
    LandmarkKey(uint16_t schema_version, uint16_t rover_id, uint32_t local_index, uint64_t descriptor_hash);

    /**
     * @brief Factory method for creating a valid LandmarkKey.
     * @return true and populated key on success; false if validation fails.
     */
    static bool try_create(uint16_t schema_version, uint16_t rover_id, uint32_t local_index,
                           uint64_t descriptor_hash, LandmarkKey& out_key) noexcept;

    /**
     * @brief Returns an explicitly invalid LandmarkKey sentinel.
     */
    static constexpr LandmarkKey invalid() noexcept {
        return LandmarkKey();
    }

    /// Accessors
    constexpr uint16_t schema_version() const noexcept { return schema_version_; }
    constexpr uint16_t rover_id() const noexcept { return rover_id_; }
    constexpr uint32_t local_index() const noexcept { return local_index_; }
    constexpr uint64_t descriptor_hash() const noexcept { return descriptor_hash_; }

    /// Validation inspector
    constexpr bool is_valid() const noexcept {
        return schema_version_ != INVALID_SCHEMA;
    }

    /// Explicit equality operator
    constexpr bool operator==(const LandmarkKey& other) const noexcept {
        return schema_version_ == other.schema_version_ &&
               rover_id_ == other.rover_id_ &&
               local_index_ == other.local_index_ &&
               descriptor_hash_ == other.descriptor_hash_;
    }

    /// Explicit inequality operator
    constexpr bool operator!=(const LandmarkKey& other) const noexcept {
        return !(*this == other);
    }

    /// Strict weak ordering for std::set, std::map, and sorting
    constexpr bool operator<(const LandmarkKey& other) const noexcept {
        if (schema_version_ != other.schema_version_) return schema_version_ < other.schema_version_;
        if (rover_id_ != other.rover_id_) return rover_id_ < other.rover_id_;
        if (local_index_ != other.local_index_) return local_index_ < other.local_index_;
        return descriptor_hash_ < other.descriptor_hash_;
    }

    constexpr bool operator<=(const LandmarkKey& other) const noexcept {
        return !(other < *this);
    }

    constexpr bool operator>(const LandmarkKey& other) const noexcept {
        return other < *this;
    }

    constexpr bool operator>=(const LandmarkKey& other) const noexcept {
        return !(*this < other);
    }

    /// Formats key into a human-readable string: "LM[s:<schema>,r:<rover>,i:<idx>,h:0x<hash>]"
    std::string to_string() const;

private:
    uint16_t schema_version_;
    uint16_t rover_id_;
    uint32_t local_index_;
    uint64_t descriptor_hash_;
};

/// Stream output operator
std::ostream& operator<<(std::ostream& os, const LandmarkKey& key);

} // namespace astromesh::core

/// Specialization of std::hash for LandmarkKey
namespace std {
template <>
struct hash<astromesh::core::LandmarkKey> {
    size_t operator()(const astromesh::core::LandmarkKey& key) const noexcept {
        // High-quality 64-bit mixer (SplitMix64 step)
        auto mix = [](uint64_t x) -> uint64_t {
            x ^= x >> 30;
            x *= 0xbf58476d1ce4e5b9ULL;
            x ^= x >> 27;
            x *= 0x94d049bb133111ebULL;
            x ^= x >> 31;
            return x;
        };

        // Combine fields deterministically
        uint64_t h1 = (static_cast<uint64_t>(key.schema_version()) << 48) |
                      (static_cast<uint64_t>(key.rover_id()) << 32) |
                      static_cast<uint64_t>(key.local_index());
        uint64_t h2 = key.descriptor_hash();

        uint64_t combined = mix(h1 ^ mix(h2 + 0x9e3779b97f4a7c15ULL));
        return static_cast<size_t>(combined);
    }
};
} // namespace std
