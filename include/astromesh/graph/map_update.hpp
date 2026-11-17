#pragma once

#include "astromesh/core/landmark_key.hpp"
#include "astromesh/core/version_vector.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace astromesh::graph {

enum class MapUpdateType : uint8_t {
    KeyframeAdded = 1,
    LandmarkObserved = 2,
    EdgeVerified = 3,
    EdgeInvalidated = 4
};

const char* to_string(MapUpdateType type) noexcept;

/**
 * @brief Immutable, versioned map mutation record with causal context.
 */
struct MapUpdate {
    static constexpr uint32_t MAGIC = 0x55504454; // 'UPDT'
    static constexpr uint8_t CURRENT_VERSION = 1;

    uint8_t schema_version{CURRENT_VERSION};
    MapUpdateType type{MapUpdateType::KeyframeAdded};
    uint16_t origin_rover_id{0};
    uint64_t sequence{0};
    uint64_t update_id{0}; // (origin << 48) | sequence
    core::VersionVector causal_context;

    // Payload Fields
    NodeId node_id{0, 0};
    NodeId target_node_id{0, 0}; // For EdgeVerified / EdgeInvalidated
    geometry::SE2 transform{0.0, 0.0, 0.0};
    core::LandmarkKey landmark_key;
    double landmark_local_x{0.0};
    double landmark_local_y{0.0};
    uint32_t inlier_count{0};
    double inlier_rmse{0.0};
    EdgeType edge_type{EdgeType::Odometry};

    static MapUpdate make_keyframe(
        uint16_t rover_id,
        uint64_t seq,
        uint32_t kf_index,
        const geometry::SE2& pose,
        const core::VersionVector& context);

    static MapUpdate make_landmark_obs(
        uint16_t rover_id,
        uint64_t seq,
        uint32_t kf_index,
        const core::LandmarkKey& key,
        double lx,
        double ly,
        const core::VersionVector& context);

    static MapUpdate make_edge_verified(
        uint16_t rover_id,
        uint64_t seq,
        const NodeId& from,
        const NodeId& to,
        const geometry::SE2& relative_pose,
        uint32_t inliers,
        double rmse,
        EdgeType etype,
        const core::VersionVector& context);

    std::vector<uint8_t> serialize() const;
    static std::optional<MapUpdate> deserialize(const uint8_t* data, size_t size);
};

} // namespace astromesh::graph

