#include "astromesh/exploration/frontier.hpp"

#include <algorithm>
#include <cmath>

namespace astromesh::exploration {

std::string to_string(FrontierStatus status) {
    switch (status) {
        case FrontierStatus::Candidate: return "Candidate";
        case FrontierStatus::Active: return "Active";
        case FrontierStatus::Claimed: return "Claimed";
        case FrontierStatus::Completed: return "Completed";
        case FrontierStatus::Expired: return "Expired";
        case FrontierStatus::Unreachable: return "Unreachable";
        default: return "Unknown";
    }
}

TopologicalFrontierDetector::TopologicalFrontierDetector(FrontierDetectorConfig config)
    : m_config(config) {}

bool TopologicalFrontierDetector::is_within_bounds(const verification::Point2D& pt) const noexcept {
    return pt.x >= m_config.min_x && pt.x <= m_config.max_x &&
           pt.y >= m_config.min_y && pt.y <= m_config.max_y;
}

uint64_t TopologicalFrontierDetector::generate_frontier_id(
    uint16_t rover_id, uint64_t seq, const verification::Point2D& pt) const noexcept {
    uint64_t h = (static_cast<uint64_t>(rover_id) << 48) | (seq & 0xFFFFFFFFULL);
    int64_t ix = static_cast<int64_t>(pt.x * 100.0);
    int64_t iy = static_cast<int64_t>(pt.y * 100.0);
    h ^= (static_cast<uint64_t>(ix) * 0xbf58476d1ce4e5b9ULL);
    h ^= (static_cast<uint64_t>(iy) * 0x94d049bb133111ebULL);
    return (h == 0) ? 1 : h;
}

bool TopologicalFrontierDetector::is_covered_by_existing_data(
    const verification::Point2D& pt,
    const graph::PoseGraph& graph,
    const std::vector<verification::Point2D>& landmarks) const noexcept {

    // 1. Check proximity to keyframe nodes
    for (const auto& [nid, node] : graph.nodes()) {
        double dx = pt.x - node.pose.x;
        double dy = pt.y - node.pose.y;
        if (std::hypot(dx, dy) < m_config.sensor_range * 0.4) {
            return true;
        }
    }

    // 2. Check proximity to mapped landmarks
    size_t nearby_lms = 0;
    for (const auto& lm : landmarks) {
        double dx = pt.x - lm.x;
        double dy = pt.y - lm.y;
        if (std::hypot(dx, dy) < m_config.sensor_range * 0.5) {
            nearby_lms++;
            if (nearby_lms >= 3) {
                return true;
            }
        }
    }

    return false;
}

std::vector<Frontier> TopologicalFrontierDetector::detect_frontiers(
    const graph::PoseGraph& graph,
    const std::vector<verification::Point2D>& landmarks,
    uint16_t rover_id,
    uint64_t current_step) {

    std::vector<Frontier> newly_detected;
    if (graph.nodes().empty()) {
        return newly_detected;
    }

    // 1. Identify peripheral nodes (degree <= 2 or boundary nodes)
    std::vector<graph::KeyframeNode> peripheral_nodes;
    for (const auto& [nid, node] : graph.nodes()) {
        if (nid.rover_id != rover_id) continue;
        auto incident = graph.incident_edges(nid);
        if (incident.size() <= 2) {
            peripheral_nodes.push_back(node);
        }
    }

    if (peripheral_nodes.empty()) {
        for (const auto& [nid, node] : graph.nodes()) {
            if (nid.rover_id == rover_id) {
                peripheral_nodes.push_back(node);
            }
        }
    }

    // 2. Radially project from peripheral nodes into unobserved angular sectors
    const double angle_step = 2.0 * M_PI / static_cast<double>(m_config.num_angular_sectors);

    for (const auto& node : peripheral_nodes) {
        for (uint32_t s = 0; s < m_config.num_angular_sectors; ++s) {
            double angle = node.pose.theta + static_cast<double>(s) * angle_step;
            double proj_x = node.pose.x + m_config.projection_distance * std::cos(angle);
            double proj_y = node.pose.y + m_config.projection_distance * std::sin(angle);
            verification::Point2D cand_pt(proj_x, proj_y);

            // Validate within environment bounds
            if (!is_within_bounds(cand_pt)) {
                continue;
            }

            // Check if already covered by known map
            if (is_covered_by_existing_data(cand_pt, graph, landmarks)) {
                continue;
            }

            // Check spatial deduplication against already tracked frontiers
            bool duplicate = false;
            for (const auto& existing : m_frontiers) {
                if (existing.status == FrontierStatus::Completed || existing.status == FrontierStatus::Expired) {
                    continue;
                }
                double dx = cand_pt.x - existing.position.x;
                double dy = cand_pt.y - existing.position.y;
                if (std::hypot(dx, dy) < m_config.frontier_cluster_radius) {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate) {
                for (const auto& nf : newly_detected) {
                    double dx = cand_pt.x - nf.position.x;
                    double dy = cand_pt.y - nf.position.y;
                    if (std::hypot(dx, dy) < m_config.frontier_cluster_radius) {
                        duplicate = true;
                        break;
                    }
                }
            }

            if (!duplicate) {
                Frontier f;
                f.frontier_id = generate_frontier_id(rover_id, m_next_seq++, cand_pt);
                f.position = cand_pt;
                f.origin_rover_id = rover_id;
                f.detected_step = current_step;
                f.ttl_steps = m_config.default_ttl_steps;
                f.status = FrontierStatus::Active;
                f.spatial_coverage_radius = m_config.sensor_range;
                f.expected_information_gain = 1.0; // Base novelty

                m_frontiers.push_back(f);
                newly_detected.push_back(f);
            }
        }
    }

    return newly_detected;
}

void TopologicalFrontierDetector::update_frontiers(
    const graph::PoseGraph& graph,
    const std::vector<verification::Point2D>& new_landmarks,
    uint64_t current_step) {

    for (auto& f : m_frontiers) {
        if (f.status == FrontierStatus::Completed || f.status == FrontierStatus::Expired) {
            continue;
        }

        // Check if covered by new graph keyframes
        for (const auto& [nid, node] : graph.nodes()) {
            double dx = f.position.x - node.pose.x;
            double dy = f.position.y - node.pose.y;
            if (std::hypot(dx, dy) < m_config.completion_radius) {
                f.status = FrontierStatus::Completed;
                break;
            }
        }

        if (f.status == FrontierStatus::Completed) {
            continue;
        }

        // Check if covered by new landmarks
        size_t observed_lms = 0;
        for (const auto& lm : new_landmarks) {
            double dx = f.position.x - lm.x;
            double dy = f.position.y - lm.y;
            if (std::hypot(dx, dy) < m_config.completion_radius) {
                observed_lms++;
                if (observed_lms >= 2) {
                    f.status = FrontierStatus::Completed;
                    break;
                }
            }
        }
    }

    prune_expired(current_step);
}

bool TopologicalFrontierDetector::mark_completed(uint64_t frontier_id) {
    auto* f = find_frontier_mut(frontier_id);
    if (f) {
        f->status = FrontierStatus::Completed;
        return true;
    }
    return false;
}

size_t TopologicalFrontierDetector::prune_expired(uint64_t current_step) {
    size_t pruned = 0;
    for (auto& f : m_frontiers) {
        if (f.status == FrontierStatus::Active || f.status == FrontierStatus::Candidate) {
            if (current_step > f.detected_step + f.ttl_steps) {
                f.status = FrontierStatus::Expired;
                pruned++;
            }
        } else if (f.status == FrontierStatus::Claimed) {
            if (current_step > f.claim_lease_expiration) {
                // Lease expired: revert to Active
                f.status = FrontierStatus::Active;
                f.claimed_by_rover = 0;
                f.claim_lease_expiration = 0;
            }
        }
    }
    return pruned;
}

bool TopologicalFrontierDetector::claim_frontier(
    uint64_t frontier_id, uint16_t rover_id, uint64_t lease_duration, uint64_t current_step) {

    auto* f = find_frontier_mut(frontier_id);
    if (!f) return false;

    if (f->status == FrontierStatus::Active ||
        (f->status == FrontierStatus::Claimed && current_step >= f->claim_lease_expiration)) {
        f->status = FrontierStatus::Claimed;
        f->claimed_by_rover = rover_id;
        f->claim_lease_expiration = current_step + lease_duration;
        return true;
    }
    return false;
}

bool TopologicalFrontierDetector::release_claim(uint64_t frontier_id, uint16_t rover_id) {
    auto* f = find_frontier_mut(frontier_id);
    if (f && f->status == FrontierStatus::Claimed && f->claimed_by_rover == rover_id) {
        f->status = FrontierStatus::Active;
        f->claimed_by_rover = 0;
        f->claim_lease_expiration = 0;
        return true;
    }
    return false;
}

const Frontier* TopologicalFrontierDetector::find_frontier(uint64_t frontier_id) const noexcept {
    for (const auto& f : m_frontiers) {
        if (f.frontier_id == frontier_id) return &f;
    }
    return nullptr;
}

Frontier* TopologicalFrontierDetector::find_frontier_mut(uint64_t frontier_id) noexcept {
    for (auto& f : m_frontiers) {
        if (f.frontier_id == frontier_id) return &f;
    }
    return nullptr;
}

void TopologicalFrontierDetector::clear() noexcept {
    m_frontiers.clear();
    m_next_seq = 1;
}

} // namespace astromesh::exploration
