#include "astromesh/core/landmark_key.hpp"

#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace astromesh::core {

LandmarkKey::LandmarkKey(uint16_t schema_version, uint16_t rover_id, uint32_t local_index, uint64_t descriptor_hash)
    : schema_version_(schema_version),
      rover_id_(rover_id),
      local_index_(local_index),
      descriptor_hash_(descriptor_hash) {
    if (schema_version_ == INVALID_SCHEMA) {
        throw std::invalid_argument("LandmarkKey cannot be constructed with schema_version 0 (reserved for invalid sentinel)");
    }
}

bool LandmarkKey::try_create(uint16_t schema_version, uint16_t rover_id, uint32_t local_index,
                             uint64_t descriptor_hash, LandmarkKey& out_key) noexcept {
    if (schema_version == INVALID_SCHEMA) {
        return false;
    }
    out_key = LandmarkKey(schema_version, rover_id, local_index, descriptor_hash);
    return true;
}

std::string LandmarkKey::to_string() const {
    if (!is_valid()) {
        return "LM[INVALID]";
    }
    std::ostringstream oss;
    oss << "LM[s:" << schema_version_
        << ",r:" << rover_id_
        << ",i:" << local_index_
        << ",h:0x" << std::hex << std::setw(16) << std::setfill('0') << descriptor_hash_
        << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const LandmarkKey& key) {
    return os << key.to_string();
}

} // namespace astromesh::core
