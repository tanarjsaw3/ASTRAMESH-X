#include "astromesh/graph/map_update.hpp"

#include <cstring>

namespace astromesh::graph {

const char* to_string(MapUpdateType type) noexcept {
    switch (type) {
        case MapUpdateType::KeyframeAdded: return "KeyframeAdded";
        case MapUpdateType::LandmarkObserved: return "LandmarkObserved";
        case MapUpdateType::EdgeVerified: return "EdgeVerified";
        case MapUpdateType::EdgeInvalidated: return "EdgeInvalidated";
    }
    return "Unknown";
}

MapUpdate MapUpdate::make_keyframe(
    uint16_t rover_id,
    uint64_t seq,
    uint32_t kf_index,
    const geometry::SE2& pose,
    const core::VersionVector& context) {

    MapUpdate u;
    u.type = MapUpdateType::KeyframeAdded;
    u.origin_rover_id = rover_id;
    u.sequence = seq;
    u.update_id = (static_cast<uint64_t>(rover_id) << 48) | (seq & 0x0000FFFFFFFFFFFFULL);
    u.causal_context = context;
    u.node_id = NodeId(rover_id, kf_index);
    u.transform = pose;
    return u;
}

MapUpdate MapUpdate::make_landmark_obs(
    uint16_t rover_id,
    uint64_t seq,
    uint32_t kf_index,
    const core::LandmarkKey& key,
    double lx,
    double ly,
    const core::VersionVector& context) {

    MapUpdate u;
    u.type = MapUpdateType::LandmarkObserved;
    u.origin_rover_id = rover_id;
    u.sequence = seq;
    u.update_id = (static_cast<uint64_t>(rover_id) << 48) | (seq & 0x0000FFFFFFFFFFFFULL);
    u.causal_context = context;
    u.node_id = NodeId(rover_id, kf_index);
    u.landmark_key = key;
    u.landmark_local_x = lx;
    u.landmark_local_y = ly;
    return u;
}

MapUpdate MapUpdate::make_edge_verified(
    uint16_t rover_id,
    uint64_t seq,
    const NodeId& from,
    const NodeId& to,
    const geometry::SE2& relative_pose,
    uint32_t inliers,
    double rmse,
    EdgeType etype,
    const core::VersionVector& context) {

    MapUpdate u;
    u.type = MapUpdateType::EdgeVerified;
    u.origin_rover_id = rover_id;
    u.sequence = seq;
    u.update_id = (static_cast<uint64_t>(rover_id) << 48) | (seq & 0x0000FFFFFFFFFFFFULL);
    u.causal_context = context;
    u.node_id = from;
    u.target_node_id = to;
    u.transform = relative_pose;
    u.inlier_count = inliers;
    u.inlier_rmse = rmse;
    u.edge_type = etype;
    return u;
}

std::vector<uint8_t> MapUpdate::serialize() const {
    // 4(MAGIC) + 1(ver) + 1(type) + 2(origin) + 8(seq) + 8(update_id) + 132(VV) + 8(n1) + 8(n2)
    // + 24(se2) + 16(lm_key) + 8(lx) + 8(ly) + 4(inliers) + 8(rmse) + 1(edge_type) + 7(pad)
    constexpr size_t FIXED_SIZE = 4 + 1 + 1 + 2 + 8 + 8 + core::VersionVector::WIRE_SIZE + 8 + 8 + 24 + 16 + 8 + 8 + 4 + 8 + 1 + 7;
    std::vector<uint8_t> buf(FIXED_SIZE, 0);

    uint32_t magic = MAGIC;
    std::memcpy(&buf[0], &magic, 4);
    buf[4] = schema_version;
    buf[5] = static_cast<uint8_t>(type);
    std::memcpy(&buf[6], &origin_rover_id, 2);
    std::memcpy(&buf[8], &sequence, 8);
    std::memcpy(&buf[16], &update_id, 8);

    size_t offset = 24;
    auto vv_bytes = causal_context.serialize();
    std::memcpy(&buf[offset], vv_bytes.data(), vv_bytes.size());
    offset += core::VersionVector::WIRE_SIZE;

    uint32_t r1 = node_id.rover_id;
    uint32_t k1 = node_id.local_index;
    uint32_t r2 = target_node_id.rover_id;
    uint32_t k2 = target_node_id.local_index;
    std::memcpy(&buf[offset], &r1, 4);
    std::memcpy(&buf[offset + 4], &k1, 4);
    std::memcpy(&buf[offset + 8], &r2, 4);
    std::memcpy(&buf[offset + 12], &k2, 4);
    offset += 16;

    double x = transform.x();
    double y = transform.y();
    double th = transform.theta();
    std::memcpy(&buf[offset], &x, 8);
    std::memcpy(&buf[offset + 8], &y, 8);
    std::memcpy(&buf[offset + 16], &th, 8);
    offset += 24;

    uint16_t lm_schema = landmark_key.schema_version();
    uint16_t lm_rover = landmark_key.rover_id();
    uint32_t lm_idx = landmark_key.local_index();
    uint64_t lm_desc = landmark_key.descriptor_hash();
    std::memcpy(&buf[offset], &lm_schema, 2);
    std::memcpy(&buf[offset + 2], &lm_rover, 2);
    std::memcpy(&buf[offset + 4], &lm_idx, 4);
    std::memcpy(&buf[offset + 8], &lm_desc, 8);
    offset += 16;

    std::memcpy(&buf[offset], &landmark_local_x, 8);
    std::memcpy(&buf[offset + 8], &landmark_local_y, 8);
    offset += 16;

    std::memcpy(&buf[offset], &inlier_count, 4);
    std::memcpy(&buf[offset + 4], &inlier_rmse, 8);
    buf[offset + 12] = static_cast<uint8_t>(edge_type);
    // remaining 7 bytes are 0 padding

    return buf;
}

std::optional<MapUpdate> MapUpdate::deserialize(const uint8_t* data, size_t size) {
    constexpr size_t FIXED_SIZE = 4 + 1 + 1 + 2 + 8 + 8 + core::VersionVector::WIRE_SIZE + 8 + 8 + 24 + 16 + 8 + 8 + 4 + 8 + 1 + 7;
    if (!data || size < FIXED_SIZE) {
        return std::nullopt;
    }

    uint32_t magic = 0;
    std::memcpy(&magic, &data[0], 4);
    if (magic != MAGIC) {
        return std::nullopt;
    }

    MapUpdate u;
    u.schema_version = data[4];
    u.type = static_cast<MapUpdateType>(data[5]);
    std::memcpy(&u.origin_rover_id, &data[6], 2);
    std::memcpy(&u.sequence, &data[8], 8);
    std::memcpy(&u.update_id, &data[16], 8);

    size_t offset = 24;
    auto vv_opt = core::VersionVector::deserialize(&data[offset], core::VersionVector::WIRE_SIZE);
    if (!vv_opt.has_value()) {
        return std::nullopt;
    }
    u.causal_context = *vv_opt;
    offset += core::VersionVector::WIRE_SIZE;

    uint32_t r1 = 0, k1 = 0, r2 = 0, k2 = 0;
    std::memcpy(&r1, &data[offset], 4);
    std::memcpy(&k1, &data[offset + 4], 4);
    std::memcpy(&r2, &data[offset + 8], 4);
    std::memcpy(&k2, &data[offset + 12], 4);
    u.node_id = NodeId(static_cast<uint16_t>(r1), k1);
    u.target_node_id = NodeId(static_cast<uint16_t>(r2), k2);
    offset += 16;

    double x = 0, y = 0, th = 0;
    std::memcpy(&x, &data[offset], 8);
    std::memcpy(&y, &data[offset + 8], 8);
    std::memcpy(&th, &data[offset + 16], 8);
    u.transform = geometry::SE2(x, y, th);
    offset += 24;

    uint16_t lm_schema = 0, lm_rover = 0;
    uint32_t lm_idx = 0;
    uint64_t lm_desc = 0;
    std::memcpy(&lm_schema, &data[offset], 2);
    std::memcpy(&lm_rover, &data[offset + 2], 2);
    std::memcpy(&lm_idx, &data[offset + 4], 4);
    std::memcpy(&lm_desc, &data[offset + 8], 8);
    if (lm_schema != 0) {
        u.landmark_key = core::LandmarkKey(lm_schema, lm_rover, lm_idx, lm_desc);
    } else {
        u.landmark_key = core::LandmarkKey();
    }
    offset += 16;

    std::memcpy(&u.landmark_local_x, &data[offset], 8);
    std::memcpy(&u.landmark_local_y, &data[offset + 8], 8);
    offset += 16;

    std::memcpy(&u.inlier_count, &data[offset], 4);
    std::memcpy(&u.inlier_rmse, &data[offset + 4], 8);
    u.edge_type = static_cast<EdgeType>(data[offset + 12]);

    return u;
}

} // namespace astromesh::graph
