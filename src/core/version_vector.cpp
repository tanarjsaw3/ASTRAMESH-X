#include "astromesh/core/version_vector.hpp"

#include <cstring>
#include <sstream>

namespace astromesh::core {

std::vector<uint8_t> VersionVector::serialize() const {
    std::vector<uint8_t> buf(WIRE_SIZE);
    uint32_t magic = MAGIC;
    std::memcpy(&buf[0], &magic, 4);

    size_t offset = 4;
    for (size_t i = 0; i < MAX_ROVERS; ++i) {
        uint64_t val = m_versions[i];
        std::memcpy(&buf[offset], &val, 8);
        offset += 8;
    }
    return buf;
}

std::optional<VersionVector> VersionVector::deserialize(const uint8_t* data, size_t size) {
    if (!data || size < WIRE_SIZE) {
        return std::nullopt;
    }

    uint32_t magic = 0;
    std::memcpy(&magic, &data[0], 4);
    if (magic != MAGIC) {
        return std::nullopt;
    }

    VersionVector vv;
    size_t offset = 4;
    for (size_t i = 0; i < MAX_ROVERS; ++i) {
        uint64_t val = 0;
        std::memcpy(&val, &data[offset], 8);
        vv.m_versions[i] = val;
        offset += 8;
    }
    return vv;
}

std::string VersionVector::to_string() const {
    std::ostringstream oss;
    oss << "<";
    bool first = true;
    for (size_t i = 0; i < MAX_ROVERS; ++i) {
        if (m_versions[i] > 0) {
            if (!first) oss << ", ";
            oss << "R" << (i + 1) << ":" << m_versions[i];
            first = false;
        }
    }
    if (first) oss << "empty";
    oss << ">";
    return oss.str();
}

} // namespace astromesh::core

