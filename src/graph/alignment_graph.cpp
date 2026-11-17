#include "astromesh/graph/alignment_graph.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>

namespace astromesh::graph {

bool AlignmentGraph::add_or_update_edge(AlignmentEdge edge) {
    if (edge.from_rover == 0 || edge.to_rover == 0 || edge.from_rover == edge.to_rover) {
        return false;
    }

    auto pair_key = std::make_pair(edge.from_rover, edge.to_rover);
    auto it = m_pair_to_edge.find(pair_key);

    if (it != m_pair_to_edge.end()) {
        uint64_t eid = it->second;
        auto& exist = m_edges[eid];
        if (edge.epoch < exist.epoch && exist.is_active) {
            return false; // Stale update
        }
        exist.transform_to_from = edge.transform_to_from;
        exist.correspondence_count = edge.correspondence_count;
        exist.inlier_count = edge.inlier_count;
        exist.inlier_rmse = edge.inlier_rmse;
        exist.confidence = edge.confidence;
        exist.epoch = edge.epoch;
        exist.is_active = true;
        return true;
    }

    uint64_t eid = m_next_edge_id++;
    edge.edge_id = eid;
    edge.is_active = true;

    m_edges[eid] = edge;
    m_pair_to_edge[pair_key] = eid;
    m_adjacency[edge.from_rover].push_back(eid);

    return true;
}

bool AlignmentGraph::invalidate_edge(uint64_t edge_id) {
    auto it = m_edges.find(edge_id);
    if (it != m_edges.end() && it->second.is_active) {
        it->second.is_active = false;
        return true;
    }
    return false;
}

bool AlignmentGraph::has_edge(uint16_t from_rover, uint16_t to_rover) const noexcept {
    auto it = m_pair_to_edge.find({from_rover, to_rover});
    if (it != m_pair_to_edge.end()) {
        auto e_it = m_edges.find(it->second);
        return e_it != m_edges.end() && e_it->second.is_active;
    }
    return false;
}

std::optional<AlignmentEdge> AlignmentGraph::get_edge(uint16_t from_rover, uint16_t to_rover) const {
    auto it = m_pair_to_edge.find({from_rover, to_rover});
    if (it != m_pair_to_edge.end()) {
        auto e_it = m_edges.find(it->second);
        if (e_it != m_edges.end() && e_it->second.is_active) {
            return e_it->second;
        }
    }
    return std::nullopt;
}

std::vector<uint16_t> AlignmentGraph::get_rovers() const {
    std::set<uint16_t> rovers;
    for (const auto& [eid, e] : m_edges) {
        if (e.is_active) {
            rovers.insert(e.from_rover);
            rovers.insert(e.to_rover);
        }
    }
    return std::vector<uint16_t>(rovers.begin(), rovers.end());
}

std::vector<std::vector<uint16_t>> AlignmentGraph::get_connected_components() const {
    auto rovers = get_rovers();
    std::set<uint16_t> unvisited(rovers.begin(), rovers.end());
    std::vector<std::vector<uint16_t>> components;

    while (!unvisited.empty()) {
        uint16_t start = *unvisited.begin();
        unvisited.erase(unvisited.begin());

        std::vector<uint16_t> comp = {start};
        std::queue<uint16_t> q;
        q.push(start);

        while (!q.empty()) {
            uint16_t curr = q.front();
            q.pop();

            for (const auto& [eid, e] : m_edges) {
                if (!e.is_active) continue;
                uint16_t nxt = 0;
                if (e.from_rover == curr) nxt = e.to_rover;
                else if (e.to_rover == curr) nxt = e.from_rover;

                if (nxt != 0 && unvisited.find(nxt) != unvisited.end()) {
                    unvisited.erase(nxt);
                    comp.push_back(nxt);
                    q.push(nxt);
                }
            }
        }
        std::sort(comp.begin(), comp.end());
        components.push_back(std::move(comp));
    }
    return components;
}

std::optional<geometry::SE2> AlignmentGraph::resolve_transform_from_anchor(uint16_t root_anchor, uint16_t target_rover) const {
    if (root_anchor == target_rover) {
        return geometry::SE2::identity();
    }

    // Shortest path with weights = 1.0 / (confidence + 1e-4)
    struct NodeState {
        uint16_t rover;
        double dist;
        geometry::SE2 accumulated_tx;
    };

    struct Compare {
        bool operator()(const NodeState& a, const NodeState& b) const {
            return a.dist > b.dist;
        }
    };

    std::priority_queue<NodeState, std::vector<NodeState>, Compare> pq;
    std::unordered_map<uint16_t, double> best_dist;

    pq.push({root_anchor, 0.0, geometry::SE2::identity()});
    best_dist[root_anchor] = 0.0;

    while (!pq.empty()) {
        auto curr = pq.top();
        pq.pop();

        if (curr.rover == target_rover) {
            return curr.accumulated_tx;
        }

        if (curr.dist > best_dist[curr.rover]) {
            continue;
        }

        for (const auto& [eid, e] : m_edges) {
            if (!e.is_active) continue;

            uint16_t neighbor = 0;
            geometry::SE2 step_tx;

            // If edge is neighbor -> curr: e.transform_to_from maps neighbor to curr
            // T_anchor_neighbor = T_anchor_curr * T_curr_neighbor
            if (e.to_rover == curr.rover) {
                neighbor = e.from_rover;
                step_tx = e.transform_to_from;
            } else if (e.from_rover == curr.rover) {
                neighbor = e.to_rover;
                step_tx = e.transform_to_from.inverse();
            }

            if (neighbor != 0) {
                double edge_weight = 1.0 / (e.confidence + 1e-4);
                double new_dist = curr.dist + edge_weight;

                if (best_dist.find(neighbor) == best_dist.end() || new_dist < best_dist[neighbor]) {
                    best_dist[neighbor] = new_dist;
                    geometry::SE2 next_acc = curr.accumulated_tx * step_tx;
                    pq.push({neighbor, new_dist, next_acc});
                }
            }
        }
    }

    return std::nullopt; // Disconnected
}

CycleAuditResult AlignmentGraph::audit_cycle_consistency(double tolerance_m, size_t max_cycle_len) const {
    CycleAuditResult res;
    auto rovers = get_rovers();

    // Audit 3-cycles
    if (max_cycle_len >= 3) {
        for (size_t i = 0; i < rovers.size(); ++i) {
            for (size_t j = i + 1; j < rovers.size(); ++j) {
                for (size_t k = j + 1; k < rovers.size(); ++k) {
                    uint16_t r1 = rovers[i];
                    uint16_t r2 = rovers[j];
                    uint16_t r3 = rovers[k];

                    auto e12_opt = get_edge(r2, r1); // T_12
                    auto e23_opt = get_edge(r3, r2); // T_23
                    auto e31_opt = get_edge(r1, r3); // T_31

                    if (e12_opt && e23_opt && e31_opt) {
                        res.evaluated_cycle_count++;

                        // Loop: T_12 * T_23 * T_31 should equal identity
                        auto loop_tx = e12_opt->transform_to_from *
                                       e23_opt->transform_to_from *
                                       e31_opt->transform_to_from;

                        double err = std::sqrt(loop_tx.x() * loop_tx.x() +
                                               loop_tx.y() * loop_tx.y() +
                                               2.0 * loop_tx.theta() * loop_tx.theta());

                        if (err > res.max_cycle_error_m) {
                            res.max_cycle_error_m = err;
                        }

                        if (err > tolerance_m) {
                            res.has_inconsistent_cycles = true;
                            res.inconsistent_cycles.push_back({r1, r2, r3});

                            // Suggest worst RMSE edge for invalidation
                            uint64_t worst_id = e12_opt->edge_id;
                            double worst_rmse = e12_opt->inlier_rmse;

                            if (e23_opt->inlier_rmse > worst_rmse) {
                                worst_rmse = e23_opt->inlier_rmse;
                                worst_id = e23_opt->edge_id;
                            }
                            if (e31_opt->inlier_rmse > worst_rmse) {
                                worst_id = e31_opt->edge_id;
                            }
                            res.suggested_invalidations.push_back(worst_id);
                        }
                    }
                }
            }
        }
    }

    return res;
}

} // namespace astromesh::graph
