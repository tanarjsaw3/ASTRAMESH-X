#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::core {

/**
 * @brief Causal Version Vector for tracking distributed map state across rovers.
 * Bounded to MAX_ROVERS (16) with 64-bit monotonic logical sequence counters.
 */
class VersionVector {
public:
    static constexpr size_t MAX_ROVERS = 16;
    static constexpr uint32_t MAGIC = 0x56564543; // 'VVEC' Little-Endian
    static constexpr size_t WIRE_SIZE = 4 + (MAX_ROVERS * 8); // 132 bytes

    VersionVector() noexcept {
        m_versions.fill(0);
    }

    uint64_t get(uint16_t rover_id) const noexcept {
        if (rover_id == 0 || rover_id > MAX_ROVERS) return 0;
        return m_versions[rover_id - 1];
    }

    void set(uint16_t rover_id, uint64_t seq) noexcept {
        if (rover_id > 0 && rover_id <= MAX_ROVERS) {
            m_versions[rover_id - 1] = seq;
        }
    }

    uint64_t increment(uint16_t rover_id) noexcept {
        if (rover_id > 0 && rover_id <= MAX_ROVERS) {
            return ++m_versions[rover_id - 1];
        }
        return 0;
    }

    bool equals(const VersionVector& other) const noexcept {
        return m_versions == other.m_versions;
    }

    bool operator==(const VersionVector& other) const noexcept {
        return equals(other);
    }

    bool operator!=(const VersionVector& other) const noexcept {
        return !equals(other);
    }

    /**
     * @brief Tests if this version vector dominates or equals another (this >= other).
     */
    bool dominates_or_equal(const VersionVector& other) const noexcept {
        for (size_t i = 0; i < MAX_ROVERS; ++i) {
            if (m_versions[i] < other.m_versions[i]) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Tests if this version vector strictly dominates another (this > other).
     */
    bool strictly_dominates(const VersionVector& other) const noexcept {
        return dominates_or_equal(other) && !equals(other);
    }

    /**
     * @brief Tests if two version vectors are concurrent (neither dominates the other).
     */
    bool is_concurrent_with(const VersionVector& other) const noexcept {
        return !dominates_or_equal(other) && !other.dominates_or_equal(*this);
    }

    /**
     * @brief Merges this vector with another by taking the element-wise maximum.
     */
    VersionVector merge_max(const VersionVector& other) const noexcept {
        VersionVector res;
        for (size_t i = 0; i < MAX_ROVERS; ++i) {
            res.m_versions[i] = std::max(m_versions[i], other.m_versions[i]);
        }
        return res;
    }

    std::vector<uint8_t> serialize() const;
    static std::optional<VersionVector> deserialize(const uint8_t* data, size_t size);
    std::string to_string() const;

    const std::array<uint64_t, MAX_ROVERS>& raw_versions() const noexcept {
        return m_versions;
    }

private:
    std::array<uint64_t, MAX_ROVERS> m_versions{0};
};

} // namespace astromesh::core

