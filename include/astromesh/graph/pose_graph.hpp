#pragma once

#include "astromesh/core/landmark_key.hpp"

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <optional>
#include <stdexcept>
#include <iosfwd>

namespace astromesh::graph {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Normalizes an angle in radians to the half-open interval [-pi, pi).
 */
inline double normalize_angle(double angle) noexcept {
    if (!std::isfinite(angle)) {
        return angle;
    }
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

/**
 * @brief Represents a 2D planar pose (SE(2)) in a right-handed Cartesian frame:
 *        - x: Forward / East displacement in meters.
 *        - y: Left / North displacement in meters.
 *        - theta: Counter-clockwise heading / yaw angle in radians, normalized to [-pi, pi).
 */
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    constexpr Pose2D() noexcept = default;
    Pose2D(double px, double py, double ptheta, bool normalize = true) noexcept
        : x(px), y(py), theta(normalize ? normalize_angle(ptheta) : ptheta) {}

    bool is_valid() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(theta);
    }

    bool operator==(const Pose2D& other) const noexcept {
        constexpr double EPS = 1e-9;
        return std::abs(x - other.x) < EPS &&
               std::abs(y - other.y) < EPS &&
               std::abs(normalize_angle(theta - other.theta)) < EPS;
    }

    bool operator!=(const Pose2D& other) const noexcept {
        return !(*this == other);
    }
};

/**
 * @brief Represents a 3x3 symmetric positive-definite information matrix
 *        (inverse covariance) for planar SE(2) relative pose constraints:
 *
 *        [ xx      xy      xtheta     ]
 *        [ xy      yy      ytheta     ]
 *        [ xtheta  ytheta  thetatheta ]
 */
struct InformationMatrix2D {
    double xx{1.0};
    double xy{0.0};
    double xtheta{0.0};
    double yy{1.0};
    double ytheta{0.0};
    double thetatheta{1.0};

    constexpr InformationMatrix2D() noexcept = default;
    constexpr InformationMatrix2D(double m_xx, double m_xy, double m_xtheta,
                                  double m_yy, double m_ytheta, double m_thetatheta) noexcept
        : xx(m_xx), xy(m_xy), xtheta(m_xtheta), yy(m_yy), ytheta(m_ytheta), thetatheta(m_thetatheta) {}

    /**
     * @brief Creates an isotropic diagonal information matrix with position and rotation weights.
     */
    static InformationMatrix2D diagonal(double pos_weight, double rot_weight) noexcept {
        return InformationMatrix2D(pos_weight, 0.0, 0.0, pos_weight, 0.0, rot_weight);
    }

    /**
     * @brief Validates that all entries are finite and the matrix is strictly positive definite.
     *        Uses Sylvester's criterion on leading principal minors:
     *        1. M1 = xx > 0
     *        2. M2 = xx * yy - xy^2 > 0
     *        3. M3 = det(M) > 0
     */
    bool is_valid() const noexcept {
        if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(xtheta) ||
            !std::isfinite(yy) || !std::isfinite(ytheta) || !std::isfinite(thetatheta)) {
            return false;
        }
        if (xx <= 0.0 || yy <= 0.0 || thetatheta <= 0.0) {
            return false;
        }
        double m2 = xx * yy - xy * xy;
        if (m2 <= 0.0) {
            return false;
        }
        double det = xx * (yy * thetatheta - ytheta * ytheta)
                   - xy * (xy * thetatheta - xtheta * ytheta)
                   + xtheta * (xy * ytheta - xtheta * yy);
        return det > 0.0;
    }
};

/**
 * @brief Globally unique, stable identifier for a keyframe node in the multi-rover swarm.
 *        Combines the originating rover ID (16-bit) and sequential local keyframe index (32-bit).
 *        rover_id == 0 is reserved as an invalid sentinel.
 */
struct NodeId {
    uint16_t rover_id{0};
    uint32_t local_index{0};

    constexpr NodeId() noexcept = default;
    constexpr NodeId(uint16_t r_id, uint32_t l_idx) noexcept
        : rover_id(r_id), local_index(l_idx) {}

    constexpr bool is_valid() const noexcept {
        return rover_id != 0;
    }

    constexpr bool operator==(const NodeId& other) const noexcept {
        return rover_id == other.rover_id && local_index == other.local_index;
    }

    constexpr bool operator!=(const NodeId& other) const noexcept {
        return !(*this == other);
    }

    constexpr bool operator<(const NodeId& other) const noexcept {
        if (rover_id != other.rover_id) return rover_id < other.rover_id;
        return local_index < other.local_index;
    }

    std::string to_string() const;
};

std::ostream& operator<<(std::ostream& os, const NodeId& id);

} // namespace astromesh::graph

namespace std {
template <>
struct hash<astromesh::graph::NodeId> {
    size_t operator()(const astromesh::graph::NodeId& id) const noexcept {
        uint64_t combined = (static_cast<uint64_t>(id.rover_id) << 32) | id.local_index;
        combined ^= combined >> 30;
        combined *= 0xbf58476d1ce4e5b9ULL;
        combined ^= combined >> 27;
        combined *= 0x94d049bb133111ebULL;
        combined ^= combined >> 31;
        return static_cast<size_t>(combined);
    }
};
} // namespace std

namespace astromesh::graph {

/**
 * @brief Represents a topological pose-graph node corresponding to a rover keyframe.
 */
struct KeyframeNode {

    NodeId id;
    Pose2D pose;                                      ///< Initial or odometric pose in rover's local map frame.
    uint64_t timestamp_ns{0};                         ///< Nanosecond acquisition timestamp (optional metadata).
    std::vector<astromesh::core::LandmarkKey> observed_landmarks; ///< Explicit references to observed landmarks.

    KeyframeNode() = default;
    KeyframeNode(const NodeId& nid, const Pose2D& p, uint64_t t_ns = 0)
        : id(nid), pose(p), timestamp_ns(t_ns) {}

    bool is_valid() const noexcept {
        return id.is_valid() && pose.is_valid();
    }
};

/**
 * @brief Classification of pose-graph edges.
 */
enum class EdgeType {
    Odometry,               ///< Sequential intra-rover kinematic/odometric link between keyframe k and k+1.
    IntraRoverLoopClosure,  ///< Loop closure constraint between non-consecutive keyframes of the SAME rover.
    InterRoverLoopClosure   ///< Cross-rover loop closure constraint connecting keyframes of TWO DIFFERENT rovers.
};

/**
 * @brief Verification status of a proposed graph constraint.
 * @note Per scientific guidelines: A proposed loop closure candidate is NOT geometrically
 *       verified merely because it exists as an edge in the graph.
 */
enum class EdgeVerificationStatus {
    Unverified, ///< Candidate proposal (e.g. from set reconciliation); awaiting metric verification.
    Verified,   ///< Successfully gated by metric consistency (e.g. RANSAC / ICP / PnP).
    Rejected    ///< Gated out as an outlier or perceptual aliasing artifact.
};

/**
 * @brief Represents a directed relative pose constraint between two keyframe nodes:
 *        T_from -> T_to, such that relative_pose = T_from^{-1} * T_to.
 */
struct PoseGraphEdge {
    uint64_t id{0};                                   ///< Unique edge ID within graph.
    NodeId from_node;                                 ///< Source keyframe identifier.
    NodeId to_node;                                   ///< Target keyframe identifier.
    EdgeType type{EdgeType::Odometry};                ///< Functional role of the edge.
    Pose2D relative_pose;                             ///< Measured relative transform from source to target.
    InformationMatrix2D information;                  ///< Constraint precision / inverse covariance.
    EdgeVerificationStatus status{EdgeVerificationStatus::Unverified}; ///< Metric verification gate.

    PoseGraphEdge() = default;
    PoseGraphEdge(uint64_t eid, const NodeId& from, const NodeId& to,
                  EdgeType etype, const Pose2D& rel_pose,
                  const InformationMatrix2D& info = InformationMatrix2D::diagonal(100.0, 1000.0),
                  EdgeVerificationStatus v_status = EdgeVerificationStatus::Unverified)
        : id(eid), from_node(from), to_node(to), type(etype),
          relative_pose(rel_pose), information(info), status(v_status) {}

    bool is_valid() const noexcept {
        return from_node.is_valid() && to_node.is_valid() &&
               from_node != to_node && // Prohibit self-edges
               relative_pose.is_valid() && information.is_valid();
    }
};

/**
 * @brief Detailed operation status returned by PoseGraph mutators.
 */
enum class GraphStatus {
    Success,
    InvalidNode,
    DuplicateNode,
    NodeNotFound,
    InvalidEdge,
    DuplicateEdge,
    SelfEdgeProhibited,
    MissingEndpointNode,
    EdgeNotFound
};

/**
 * @brief Topological Pose-Graph data structure for multi-rover collaborative mapping.
 *
 * Characteristics:
 * - Deterministic, container-managed storage for KeyframeNode and PoseGraphEdge entities.
 * - Enforces node existence before edge insertion.
 * - Strictly rejects self-edges (from_node == to_node).
 * - Enforces duplicate node ID and duplicate edge ID protection.
 * - Distinguishes between Odometry, Intra-Rover Loop Closures, and Inter-Rover Loop Closures.
 * - Maintains topological adjacency index for fast neighbor and incident-edge queries.
 * - Does NOT perform pose optimization or assume candidate loop closures are verified.
 */
class PoseGraph {
public:
    PoseGraph() = default;
    ~PoseGraph() = default;

    // --- Node Mutators ---

    /**
     * @brief Adds a keyframe node with detailed status reporting.
     */
    GraphStatus add_node_detailed(const KeyframeNode& node);

    /**
     * @brief Convenience node addition returning true on success.
     */
    bool add_node(const KeyframeNode& node);

    /**
     * @brief Updates the estimated pose of an existing keyframe node.
     * @return true if node exists and pose is valid; false otherwise.
     */
    bool update_node_pose(const NodeId& id, const Pose2D& new_pose);

    /**
     * @brief Removes a keyframe node and all incident edges.
     * @return true if node was present and removed.
     */
    bool remove_node(const NodeId& id);

    // --- Node Queries ---

    bool has_node(const NodeId& id) const noexcept;
    const KeyframeNode* find_node(const NodeId& id) const noexcept;
    KeyframeNode* find_node_mut(const NodeId& id) noexcept;
    const KeyframeNode& get_node(const NodeId& id) const;

    size_t num_nodes() const noexcept { return nodes_.size(); }
    const std::unordered_map<NodeId, KeyframeNode>& nodes() const noexcept { return nodes_; }

    /**
     * @brief Enumerates all unique rover IDs represented in this pose-graph.
     */
    std::vector<uint16_t> rovers() const;

    /**
     * @brief Retrieves all node IDs originating from a specific rover.
     */
    std::vector<NodeId> nodes_for_rover(uint16_t rover_id) const;

    // --- Edge Mutators ---

    /**
     * @brief Adds a relative pose constraint edge with detailed status reporting.
     *        If edge.id == 0, an internal unique edge ID will be automatically assigned.
     */
    GraphStatus add_edge_detailed(PoseGraphEdge edge);

    /**
     * @brief Convenience edge addition returning true on success.
     */
    bool add_edge(PoseGraphEdge edge);

    /**
     * @brief Removes an edge by its identifier.
     */
    bool remove_edge(uint64_t edge_id);

    /**
     * @brief Updates the verification status and optional refined relative pose of an existing edge.
     * @param edge_id The identifier of the edge to update.
     * @param new_status The new EdgeVerificationStatus (e.g. Verified or Rejected).
     * @param verified_relative_pose Optional refined relative pose from geometric verification.
     * @return true if edge was found and updated; false if edge_id does not exist or pose is non-finite.
     */
    bool update_edge_verification(uint64_t edge_id, EdgeVerificationStatus new_status,
                                  const std::optional<Pose2D>& verified_relative_pose = std::nullopt);

    // --- Edge Queries ---


    bool has_edge(uint64_t edge_id) const noexcept;
    const PoseGraphEdge* find_edge(uint64_t edge_id) const noexcept;
    const PoseGraphEdge& get_edge(uint64_t edge_id) const;

    size_t num_edges() const noexcept { return edges_.size(); }
    size_t num_edges_of_type(EdgeType type) const noexcept;

    const std::unordered_map<uint64_t, PoseGraphEdge>& edges() const noexcept { return edges_; }

    /**
     * @brief Returns edge IDs of all edges incident to the specified node (incoming or outgoing).
     */
    std::vector<uint64_t> incident_edges(const NodeId& id) const;

    /**
     * @brief Finds all edges connecting node u and node v (in either direction).
     */
    std::vector<PoseGraphEdge> edges_between(const NodeId& u, const NodeId& v) const;

    // --- Graph Maintenance ---

    void clear() noexcept;
    bool empty() const noexcept { return nodes_.empty() && edges_.empty(); }

private:
    std::unordered_map<NodeId, KeyframeNode> nodes_;
    std::unordered_map<uint64_t, PoseGraphEdge> edges_;

    // Adjacency index: maps NodeId -> list of incident edge IDs
    std::unordered_map<NodeId, std::vector<uint64_t>> adjacency_;

    uint64_t next_auto_edge_id_{1};
};

} // namespace astromesh::graph
