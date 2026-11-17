#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/cord_integrator.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace astromesh::tests::support {

/**
 * @brief Self-contained synthetic benchmark dataset container.
 */
struct SyntheticDataset {
    std::string name;
    graph::PoseGraph graph;
    std::unordered_map<graph::NodeId, geometry::SE2> ground_truth_poses;
    std::unordered_map<graph::NodeId, geometry::SE2> initial_poses;
    size_t num_odometry_edges{0};
    size_t num_verified_loop_closures{0};
    size_t num_unverified_loop_closures{0};
    size_t num_rejected_loop_closures{0};
    uint32_t seed{0};
};

/**
 * @brief Helper to create standard isotropic diagonal InformationMatrix2D.
 */
inline graph::InformationMatrix2D make_info_matrix(double trans_std, double rot_std) noexcept {
    const double w_trans = 1.0 / (trans_std * trans_std);
    const double w_rot = 1.0 / (rot_std * rot_std);
    return graph::InformationMatrix2D::diagonal(w_trans, w_rot);
}

inline bool add_node_helper(graph::PoseGraph& g, const graph::NodeId& id, const graph::Pose2D& p, uint64_t t_ns = 0) {
    return g.add_node(graph::KeyframeNode(id, p, t_ns));
}

inline bool add_odometry_helper(graph::PoseGraph& g, const graph::NodeId& from, const graph::NodeId& to, const graph::Pose2D& p, const graph::InformationMatrix2D& info) {
    return g.add_edge(graph::PoseGraphEdge(0, from, to, graph::EdgeType::Odometry, p, info, graph::EdgeVerificationStatus::Verified));
}

inline bool add_loop_closure_helper(graph::PoseGraph& g, const graph::NodeId& from, const graph::NodeId& to, const graph::Pose2D& p, const graph::InformationMatrix2D& info, graph::EdgeVerificationStatus status) {
    const graph::EdgeType etype = (from.rover_id == to.rover_id) ? graph::EdgeType::IntraRoverLoopClosure : graph::EdgeType::InterRoverLoopClosure;
    return g.add_edge(graph::PoseGraphEdge(0, from, to, etype, p, info, status));
}

/**
 * @brief Generates a straight planar trajectory with noisy odometry and dead-reckoning initial estimates.
 */
inline SyntheticDataset generate_straight_chain(
    size_t num_nodes = 20,
    double step_distance = 1.0,
    double odom_noise_trans = 0.02,
    double odom_noise_rot = 0.01,
    double init_noise_trans = 0.05,
    double init_noise_rot = 0.02,
    uint32_t seed = 42) {

    SyntheticDataset ds;
    ds.name = "StraightChain_" + std::to_string(num_nodes);
    ds.seed = seed;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_t(0.0, odom_noise_trans);
    std::normal_distribution<double> dist_odom_r(0.0, odom_noise_rot);
    std::normal_distribution<double> dist_init_t(0.0, init_noise_trans);
    std::normal_distribution<double> dist_init_r(0.0, init_noise_rot);

    const uint16_t rover_id = 1;

    // Ground truth poses
    for (size_t i = 0; i < num_nodes; ++i) {
        graph::NodeId nid(rover_id, static_cast<uint32_t>(i));
        geometry::SE2 gt(static_cast<double>(i) * step_distance, 0.0, 0.0);
        ds.ground_truth_poses[nid] = gt;

        // Noisy initial estimate (anchored at node 0)
        if (i == 0) {
            ds.initial_poses[nid] = gt;
        } else {
            geometry::SE2 noisy(gt.x() + dist_init_t(rng),
                                gt.y() + dist_init_t(rng),
                                gt.theta() + dist_init_r(rng));
            ds.initial_poses[nid] = noisy;
        }

        add_node_helper(ds.graph, nid, ds.initial_poses[nid].to_pose2d(), static_cast<uint64_t>(i * 1000));
    }

    const auto info = make_info_matrix(std::max(1e-3, odom_noise_trans), std::max(1e-3, odom_noise_rot));

    // Odometry constraints
    for (size_t i = 0; i + 1 < num_nodes; ++i) {
        graph::NodeId from(rover_id, static_cast<uint32_t>(i));
        graph::NodeId to(rover_id, static_cast<uint32_t>(i + 1));

        // True relative motion: Z_true = X_i^{-1} * X_{i+1}
        geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];

        // Add measurement noise in body frame
        geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
        geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);

        add_odometry_helper(ds.graph, from, to, z_meas.to_pose2d(), info);
        ds.num_odometry_edges++;
    }

    return ds;
}

/**
 * @brief Generates a circular closed-loop trajectory with noisy odometry and a verified loop closure.
 */
inline SyntheticDataset generate_closed_loop(
    size_t num_nodes = 20,
    double radius = 5.0,
    double odom_noise_trans = 0.02,
    double odom_noise_rot = 0.01,
    double init_noise_trans = 0.05,
    double init_noise_rot = 0.02,
    bool add_loop_closure = true,
    uint32_t seed = 42) {

    SyntheticDataset ds;
    ds.name = "ClosedLoop_" + std::to_string(num_nodes);
    ds.seed = seed;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_t(0.0, odom_noise_trans);
    std::normal_distribution<double> dist_odom_r(0.0, odom_noise_rot);
    std::normal_distribution<double> dist_init_t(0.0, init_noise_trans);
    std::normal_distribution<double> dist_init_r(0.0, init_noise_rot);

    const uint16_t rover_id = 1;
    const double d_phi = 2.0 * M_PI / static_cast<double>(num_nodes);

    for (size_t i = 0; i < num_nodes; ++i) {
        graph::NodeId nid(rover_id, static_cast<uint32_t>(i));
        const double phi = static_cast<double>(i) * d_phi;

        // Heading is tangent to the circle
        const double x = radius * std::cos(phi);
        const double y = radius * std::sin(phi);
        const double theta = phi + 0.5 * M_PI;

        geometry::SE2 gt(x, y, theta);
        ds.ground_truth_poses[nid] = gt;

        if (i == 0) {
            ds.initial_poses[nid] = gt; // Fixed anchor
        } else {
            geometry::SE2 noisy(gt.x() + dist_init_t(rng),
                                gt.y() + dist_init_t(rng),
                                gt.theta() + dist_init_r(rng));
            ds.initial_poses[nid] = noisy;
        }

        add_node_helper(ds.graph, nid, ds.initial_poses[nid].to_pose2d(), static_cast<uint64_t>(i * 1000));
    }

    const auto odom_info = make_info_matrix(std::max(1e-3, odom_noise_trans), std::max(1e-3, odom_noise_rot));

    // Odometry chain along circumference
    for (size_t i = 0; i + 1 < num_nodes; ++i) {
        graph::NodeId from(rover_id, static_cast<uint32_t>(i));
        graph::NodeId to(rover_id, static_cast<uint32_t>(i + 1));

        geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
        geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
        geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);

        add_odometry_helper(ds.graph, from, to, z_meas.to_pose2d(), odom_info);
        ds.num_odometry_edges++;
    }

    // Loop closure edge closing the loop: node (num_nodes - 1) back to node 0
    if (add_loop_closure) {
        graph::NodeId from(rover_id, static_cast<uint32_t>(num_nodes - 1));
        graph::NodeId to(rover_id, 0);

        geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
        geometry::Twist2D noise(dist_odom_t(rng) * 0.5, dist_odom_t(rng) * 0.5, dist_odom_r(rng) * 0.5);
        geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);

        const auto lc_info = make_info_matrix(std::max(1e-3, odom_noise_trans * 0.5),
                                              std::max(1e-3, odom_noise_rot * 0.5));

        add_loop_closure_helper(ds.graph, from, to, z_meas.to_pose2d(), lc_info,
                                graph::EdgeVerificationStatus::Verified);
        ds.num_verified_loop_closures++;
    }

    return ds;
}

/**
 * @brief Generates a 2D Manhattan grid trajectory with horizontal/vertical tracks and cross loop closures.
 */
inline SyntheticDataset generate_grid_manhattan(
    size_t rows = 4,
    size_t cols = 4,
    double step_size = 2.0,
    double odom_noise_trans = 0.02,
    double odom_noise_rot = 0.01,
    double init_noise_trans = 0.05,
    double init_noise_rot = 0.02,
    double loop_closure_prob = 0.5,
    uint32_t seed = 42) {

    SyntheticDataset ds;
    ds.name = "GridManhattan_" + std::to_string(rows) + "x" + std::to_string(cols);
    ds.seed = seed;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_t(0.0, odom_noise_trans);
    std::normal_distribution<double> dist_odom_r(0.0, odom_noise_rot);
    std::normal_distribution<double> dist_init_t(0.0, init_noise_trans);
    std::normal_distribution<double> dist_init_r(0.0, init_noise_rot);
    std::uniform_real_distribution<double> dist_prob(0.0, 1.0);

    const uint16_t rover_id = 1;
    const size_t total_nodes = rows * cols;

    auto get_node_id = [cols, rover_id](size_t r, size_t c) {
        return graph::NodeId(rover_id, static_cast<uint32_t>(r * cols + c));
    };

    // Ground truth and initial poses
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            graph::NodeId nid = get_node_id(r, c);
            const double x = static_cast<double>(c) * step_size;
            const double y = static_cast<double>(r) * step_size;
            const double theta = (r % 2 == 0) ? 0.0 : M_PI; // Snake traversal

            geometry::SE2 gt(x, y, theta);
            ds.ground_truth_poses[nid] = gt;

            if (r == 0 && c == 0) {
                ds.initial_poses[nid] = gt;
            } else {
                geometry::SE2 noisy(gt.x() + dist_init_t(rng),
                                    gt.y() + dist_init_t(rng),
                                    gt.theta() + dist_init_r(rng));
                ds.initial_poses[nid] = noisy;
            }

            add_node_helper(ds.graph, nid, ds.initial_poses[nid].to_pose2d(), static_cast<uint64_t>(nid.local_index * 1000));
        }
    }

    const auto odom_info = make_info_matrix(std::max(1e-3, odom_noise_trans), std::max(1e-3, odom_noise_rot));

    // Odometry snake chain traversing the grid
    std::vector<graph::NodeId> snake_order;
    snake_order.reserve(total_nodes);
    for (size_t r = 0; r < rows; ++r) {
        if (r % 2 == 0) {
            for (size_t c = 0; c < cols; ++c) snake_order.push_back(get_node_id(r, c));
        } else {
            for (int c = static_cast<int>(cols) - 1; c >= 0; --c) snake_order.push_back(get_node_id(r, static_cast<size_t>(c)));
        }
    }

    for (size_t i = 0; i + 1 < snake_order.size(); ++i) {
        const auto& from = snake_order[i];
        const auto& to = snake_order[i + 1];

        geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
        geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
        geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);

        add_odometry_helper(ds.graph, from, to, z_meas.to_pose2d(), odom_info);
        ds.num_odometry_edges++;
    }

    // Cross grid loop closures (vertical links between adjacent rows)
    const auto lc_info = make_info_matrix(std::max(1e-3, odom_noise_trans * 0.7),
                                          std::max(1e-3, odom_noise_rot * 0.7));

    for (size_t r = 0; r + 1 < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            if (dist_prob(rng) < loop_closure_prob) {
                graph::NodeId from = get_node_id(r, c);
                graph::NodeId to = get_node_id(r + 1, c);

                geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
                geometry::Twist2D noise(dist_odom_t(rng) * 0.7, dist_odom_t(rng) * 0.7, dist_odom_r(rng) * 0.7);
                geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);

                add_loop_closure_helper(ds.graph, from, to, z_meas.to_pose2d(), lc_info,
                                        graph::EdgeVerificationStatus::Verified);
                ds.num_verified_loop_closures++;
            }
        }
    }

    return ds;
}

/**
 * @brief Generates multiple disconnected subgraphs (e.g. rovers without inter-rover contact).
 */
inline SyntheticDataset generate_multi_component_graph(
    size_t num_components = 2,
    size_t nodes_per_component = 10,
    uint32_t seed = 42) {

    SyntheticDataset ds;
    ds.name = "MultiComponent_" + std::to_string(num_components) + "x" + std::to_string(nodes_per_component);
    ds.seed = seed;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_t(0.0, 0.02);
    std::normal_distribution<double> dist_odom_r(0.0, 0.01);

    const auto odom_info = make_info_matrix(0.02, 0.01);

    for (size_t comp = 0; comp < num_components; ++comp) {
        const uint16_t rover_id = static_cast<uint16_t>(comp + 1);
        const double origin_y = static_cast<double>(comp) * 20.0;

        for (size_t i = 0; i < nodes_per_component; ++i) {
            graph::NodeId nid(rover_id, static_cast<uint32_t>(i));
            geometry::SE2 gt(static_cast<double>(i) * 1.5, origin_y, 0.0);
            ds.ground_truth_poses[nid] = gt;

            // Anchor origin per component
            if (i == 0) {
                ds.initial_poses[nid] = gt;
            } else {
                geometry::SE2 noisy(gt.x() + dist_odom_t(rng) * 2.0,
                                    gt.y() + dist_odom_t(rng) * 2.0,
                                    gt.theta() + dist_odom_r(rng) * 2.0);
                ds.initial_poses[nid] = noisy;
            }

            add_node_helper(ds.graph, nid, ds.initial_poses[nid].to_pose2d(), static_cast<uint64_t>(i * 1000));
        }

        // Intra-component odometry
        for (size_t i = 0; i + 1 < nodes_per_component; ++i) {
            graph::NodeId from(rover_id, static_cast<uint32_t>(i));
            graph::NodeId to(rover_id, static_cast<uint32_t>(i + 1));

            geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
            geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
            geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);

            add_odometry_helper(ds.graph, from, to, z_meas.to_pose2d(), odom_info);
            ds.num_odometry_edges++;
        }
    }

    return ds;
}

/**
 * @brief Generates a graph containing verified, unverified, and rejected loop closures to test edge gating.
 */
inline SyntheticDataset generate_graph_with_unverified_and_rejected_closures(
    size_t num_nodes = 15,
    uint32_t seed = 42) {

    SyntheticDataset ds = generate_closed_loop(num_nodes, 4.0, 0.02, 0.01, 0.05, 0.02, true, seed);
    ds.name = "LoopClosureGatingTest_" + std::to_string(num_nodes);

    const uint16_t rover_id = 1;
    const auto lc_info = make_info_matrix(0.02, 0.01);

    // 1. Add an UNVERIFIED loop closure edge (should be ignored by objective and solver)
    graph::NodeId from_unver(rover_id, 2);
    graph::NodeId to_unver(rover_id, 8);
    geometry::SE2 bogus_meas1(100.0, -50.0, 1.5); // Wild measurement
    add_loop_closure_helper(ds.graph, from_unver, to_unver, bogus_meas1.to_pose2d(), lc_info,
                            graph::EdgeVerificationStatus::Unverified);
    ds.num_unverified_loop_closures++;

    // 2. Add a REJECTED loop closure edge (should be ignored by objective and solver)
    graph::NodeId from_rej(rover_id, 4);
    graph::NodeId to_rej(rover_id, 11);
    geometry::SE2 bogus_meas2(-80.0, 99.0, -2.1); // Outlier measurement
    add_loop_closure_helper(ds.graph, from_rej, to_rej, bogus_meas2.to_pose2d(), lc_info,
                            graph::EdgeVerificationStatus::Rejected);
    ds.num_rejected_loop_closures++;

    return ds;
}

/**
 * @brief Multi-rover partitioned synthetic dataset container for distributed benchmarks.
 */
struct MultiRoverDataset {
    std::string name;
    uint32_t num_rovers{2};
    graph::PoseGraph full_graph;
    std::unordered_map<graph::NodeId, geometry::SE2> ground_truth_poses;
    std::unordered_map<graph::NodeId, geometry::SE2> initial_poses;
    std::unordered_map<uint32_t, graph::PoseGraph> subgraphs;
    std::unordered_map<uint32_t, std::unordered_map<graph::NodeId, optimization::NodeDynamicsState>> initial_states_by_rover;
    std::vector<graph::NodeId> fixed_anchors;
    size_t num_odometry_edges{0};
    size_t num_verified_loop_closures{0};
    uint32_t seed{42};
};

inline void partition_multi_rover_dataset(MultiRoverDataset& ds) {
    ds.subgraphs.clear();
    ds.initial_states_by_rover.clear();

    for (uint32_t r = 1; r <= ds.num_rovers; ++r) {
        ds.subgraphs[r] = graph::PoseGraph();
        ds.initial_states_by_rover[r] = {};
    }

    // Add local nodes to respective subgraphs
    for (const auto& [nid, pose] : ds.initial_poses) {
        const uint32_t r = nid.rover_id;
        if (ds.subgraphs.find(r) != ds.subgraphs.end()) {
            add_node_helper(ds.subgraphs[r], nid, pose.to_pose2d());
            ds.initial_states_by_rover[r][nid] = optimization::NodeDynamicsState(pose, geometry::Twist2D());
        }
    }

    // Distribute edges
    for (const auto& [edge_id, edge] : ds.full_graph.edges()) {
        const uint32_t from_r = edge.from_node.rover_id;
        const uint32_t to_r = edge.to_node.rover_id;

        if (from_r == to_r) {
            // Local edge
            ds.subgraphs[from_r].add_edge(edge);
        } else {
            // Inter-rover edge: add to both rovers' subgraphs, adding remote ghost node if absent
            for (uint32_t r : {from_r, to_r}) {
                const graph::NodeId ghost = (r == from_r) ? edge.to_node : edge.from_node;
                if (!ds.subgraphs[r].has_node(ghost)) {
                    add_node_helper(ds.subgraphs[r], ghost, ds.initial_poses.at(ghost).to_pose2d());
                }
                ds.subgraphs[r].add_edge(edge);
            }
        }
    }
}

/**
 * @brief Generates a multi-rover straight-chain trajectory with boundary inter-rover loop closures.
 */
inline MultiRoverDataset generate_multi_rover_straight_chain(
    uint32_t num_rovers = 2,
    size_t nodes_per_rover = 10,
    double step_distance = 1.0,
    double odom_noise_trans = 0.08,
    double odom_noise_rot = 0.04,
    double init_noise_trans = 0.10,
    double init_noise_rot = 0.05,
    uint32_t seed = 42) {

    MultiRoverDataset ds;
    ds.name = "MultiRoverChain_" + std::to_string(num_rovers) + "x" + std::to_string(nodes_per_rover);
    ds.num_rovers = num_rovers;
    ds.seed = seed;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_t(0.0, odom_noise_trans);
    std::normal_distribution<double> dist_odom_r(0.0, odom_noise_rot);
    std::normal_distribution<double> dist_init_t(0.0, init_noise_trans);
    std::normal_distribution<double> dist_init_r(0.0, init_noise_rot);

    const auto odom_info = make_info_matrix(std::max(1e-3, odom_noise_trans), std::max(1e-3, odom_noise_rot));
    const auto lc_info = make_info_matrix(std::max(1e-3, odom_noise_trans * 0.8), std::max(1e-3, odom_noise_rot * 0.8));

    for (uint32_t r = 1; r <= num_rovers; ++r) {
        for (size_t i = 0; i < nodes_per_rover; ++i) {
            graph::NodeId nid(r, static_cast<uint32_t>(i));
            const double global_idx = static_cast<double>((r - 1) * nodes_per_rover + i);
            geometry::SE2 gt(global_idx * step_distance, 0.0, 0.0);
            ds.ground_truth_poses[nid] = gt;

            if (r == 1 && i == 0) {
                ds.initial_poses[nid] = gt;
                ds.fixed_anchors.push_back(nid);
            } else {
                geometry::SE2 noisy(gt.x() + dist_init_t(rng),
                                    gt.y() + dist_init_t(rng),
                                    gt.theta() + dist_init_r(rng));
                ds.initial_poses[nid] = noisy;
            }
            add_node_helper(ds.full_graph, nid, ds.initial_poses[nid].to_pose2d(), static_cast<uint64_t>(global_idx * 1000));
        }

        // Intra-rover odometry
        for (size_t i = 0; i + 1 < nodes_per_rover; ++i) {
            graph::NodeId from(r, static_cast<uint32_t>(i));
            graph::NodeId to(r, static_cast<uint32_t>(i + 1));
            geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
            geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
            geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
            add_odometry_helper(ds.full_graph, from, to, z_meas.to_pose2d(), odom_info);
            ds.num_odometry_edges++;
        }
    }

    // Inter-rover boundary loop closures connecting consecutive rovers
    for (uint32_t r = 1; r < num_rovers; ++r) {
        graph::NodeId from(r, static_cast<uint32_t>(nodes_per_rover - 1));
        graph::NodeId to(r + 1, 0);
        geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
        geometry::Twist2D noise(dist_odom_t(rng) * 0.5, dist_odom_t(rng) * 0.5, dist_odom_r(rng) * 0.5);
        geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
        add_loop_closure_helper(ds.full_graph, from, to, z_meas.to_pose2d(), lc_info, graph::EdgeVerificationStatus::Verified);
        ds.num_verified_loop_closures++;
    }

    partition_multi_rover_dataset(ds);
    return ds;
}

/**
 * @brief Generates a multi-rover Manhattan grid trajectory partitioned across rovers.
 */
inline MultiRoverDataset generate_multi_rover_grid_manhattan(
    uint32_t num_rovers = 2,
    size_t rows = 4,
    size_t cols_per_rover = 3,
    double step_size = 2.0,
    double odom_noise_trans = 0.08,
    double odom_noise_rot = 0.04,
    double init_noise_trans = 0.10,
    double init_noise_rot = 0.05,
    uint32_t seed = 42) {

    MultiRoverDataset ds;
    ds.name = "MultiRoverGrid_" + std::to_string(num_rovers) + "R_" + std::to_string(rows) + "x" + std::to_string(cols_per_rover);
    ds.num_rovers = num_rovers;
    ds.seed = seed;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_t(0.0, odom_noise_trans);
    std::normal_distribution<double> dist_odom_r(0.0, odom_noise_rot);
    std::normal_distribution<double> dist_init_t(0.0, init_noise_trans);
    std::normal_distribution<double> dist_init_r(0.0, init_noise_rot);

    const auto odom_info = make_info_matrix(std::max(1e-3, odom_noise_trans), std::max(1e-3, odom_noise_rot));
    const auto lc_info = make_info_matrix(std::max(1e-3, odom_noise_trans * 0.8), std::max(1e-3, odom_noise_rot * 0.8));

    auto get_node_id = [cols_per_rover](uint32_t r, size_t row, size_t c) {
        return graph::NodeId(r, static_cast<uint32_t>(row * cols_per_rover + c));
    };

    for (uint32_t r = 1; r <= num_rovers; ++r) {
        const double x_offset = static_cast<double>(r - 1) * static_cast<double>(cols_per_rover) * step_size;
        for (size_t row = 0; row < rows; ++row) {
            for (size_t col = 0; col < cols_per_rover; ++col) {
                graph::NodeId nid = get_node_id(r, row, col);
                const double x = x_offset + static_cast<double>(col) * step_size;
                const double y = static_cast<double>(row) * step_size;
                const double theta = (row % 2 == 0) ? 0.0 : M_PI;

                geometry::SE2 gt(x, y, theta);
                ds.ground_truth_poses[nid] = gt;

                if (r == 1 && row == 0 && col == 0) {
                    ds.initial_poses[nid] = gt;
                    ds.fixed_anchors.push_back(nid);
                } else {
                    geometry::SE2 noisy(gt.x() + dist_init_t(rng),
                                        gt.y() + dist_init_t(rng),
                                        gt.theta() + dist_init_r(rng));
                    ds.initial_poses[nid] = noisy;
                }
                add_node_helper(ds.full_graph, nid, ds.initial_poses[nid].to_pose2d());
            }
        }

        // Horizontal intra-rover odometry
        for (size_t row = 0; row < rows; ++row) {
            for (size_t col = 0; col + 1 < cols_per_rover; ++col) {
                graph::NodeId from = get_node_id(r, row, col);
                graph::NodeId to = get_node_id(r, row, col + 1);
                geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
                geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
                geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
                add_odometry_helper(ds.full_graph, from, to, z_meas.to_pose2d(), odom_info);
                ds.num_odometry_edges++;
            }
        }

        // Vertical intra-rover loop closures
        for (size_t col = 0; col < cols_per_rover; ++col) {
            for (size_t row = 0; row + 1 < rows; ++row) {
                graph::NodeId from = get_node_id(r, row, col);
                graph::NodeId to = get_node_id(r, row + 1, col);
                geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
                geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
                geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
                add_loop_closure_helper(ds.full_graph, from, to, z_meas.to_pose2d(), lc_info, graph::EdgeVerificationStatus::Verified);
                ds.num_verified_loop_closures++;
            }
        }
    }

    // Inter-rover boundary loop closures across columns for every row
    for (uint32_t r = 1; r < num_rovers; ++r) {
        for (size_t row = 0; row < rows; ++row) {
            graph::NodeId from = get_node_id(r, row, cols_per_rover - 1);
            graph::NodeId to = get_node_id(r + 1, row, 0);
            geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
            geometry::Twist2D noise(dist_odom_t(rng) * 0.5, dist_odom_t(rng) * 0.5, dist_odom_r(rng) * 0.5);
            geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
            add_loop_closure_helper(ds.full_graph, from, to, z_meas.to_pose2d(), lc_info, graph::EdgeVerificationStatus::Verified);
            ds.num_verified_loop_closures++;
        }
    }

    partition_multi_rover_dataset(ds);
    return ds;
}

/**
 * @brief Generates a multi-rover circular loop with multiple inter-rover loop closures.
 */
inline MultiRoverDataset generate_multi_rover_circular_loop(
    uint32_t num_rovers = 2,
    size_t nodes_per_rover = 10,
    double radius = 5.0,
    double odom_noise_trans = 0.08,
    double odom_noise_rot = 0.04,
    double init_noise_trans = 0.10,
    double init_noise_rot = 0.05,
    uint32_t seed = 42) {

    MultiRoverDataset ds;
    ds.name = "MultiRoverLoop_" + std::to_string(num_rovers) + "R_" + std::to_string(nodes_per_rover) + "N";
    ds.num_rovers = num_rovers;
    ds.seed = seed;

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> dist_odom_t(0.0, odom_noise_trans);
    std::normal_distribution<double> dist_odom_r(0.0, odom_noise_rot);
    std::normal_distribution<double> dist_init_t(0.0, init_noise_trans);
    std::normal_distribution<double> dist_init_r(0.0, init_noise_rot);

    const auto odom_info = make_info_matrix(std::max(1e-3, odom_noise_trans), std::max(1e-3, odom_noise_rot));
    const auto lc_info = make_info_matrix(std::max(1e-3, odom_noise_trans * 0.5), std::max(1e-3, odom_noise_rot * 0.5));

    const size_t total_nodes = num_rovers * nodes_per_rover;
    const double angle_step = (2.0 * M_PI) / static_cast<double>(total_nodes);

    for (uint32_t r = 1; r <= num_rovers; ++r) {
        for (size_t i = 0; i < nodes_per_rover; ++i) {
            graph::NodeId nid(r, static_cast<uint32_t>(i));
            const size_t global_idx = (r - 1) * nodes_per_rover + i;
            const double angle = static_cast<double>(global_idx) * angle_step;

            const double x = radius * std::cos(angle);
            const double y = radius * std::sin(angle);
            const double theta = graph::normalize_angle(angle + M_PI / 2.0);

            geometry::SE2 gt(x, y, theta);
            ds.ground_truth_poses[nid] = gt;

            if (r == 1 && i == 0) {
                ds.initial_poses[nid] = gt;
                ds.fixed_anchors.push_back(nid);
            } else {
                geometry::SE2 noisy(gt.x() + dist_init_t(rng),
                                    gt.y() + dist_init_t(rng),
                                    gt.theta() + dist_init_r(rng));
                ds.initial_poses[nid] = noisy;
            }
            add_node_helper(ds.full_graph, nid, ds.initial_poses[nid].to_pose2d(), static_cast<uint64_t>(global_idx * 1000));
        }

        // Intra-rover odometry along each arc
        for (size_t i = 0; i + 1 < nodes_per_rover; ++i) {
            graph::NodeId from(r, static_cast<uint32_t>(i));
            graph::NodeId to(r, static_cast<uint32_t>(i + 1));
            geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
            geometry::Twist2D noise(dist_odom_t(rng), dist_odom_t(rng), dist_odom_r(rng));
            geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
            add_odometry_helper(ds.full_graph, from, to, z_meas.to_pose2d(), odom_info);
            ds.num_odometry_edges++;
        }
    }

    // Inter-rover boundary loop closures connecting the arcs around the circle
    for (uint32_t r = 1; r <= num_rovers; ++r) {
        const uint32_t next_r = (r % num_rovers) + 1;
        graph::NodeId from(r, static_cast<uint32_t>(nodes_per_rover - 1));
        graph::NodeId to(next_r, 0);
        geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
        geometry::Twist2D noise(dist_odom_t(rng) * 0.5, dist_odom_t(rng) * 0.5, dist_odom_r(rng) * 0.5);
        geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
        add_loop_closure_helper(ds.full_graph, from, to, z_meas.to_pose2d(), lc_info, graph::EdgeVerificationStatus::Verified);
        ds.num_verified_loop_closures++;
    }

    // Cross-diameter loop closure between Rover 1 mid-node and Rover (num_rovers/2 + 1) mid-node
    if (num_rovers >= 2) {
        const uint32_t opp_r = (num_rovers / 2) + 1;
        graph::NodeId from(1, static_cast<uint32_t>(nodes_per_rover / 2));
        graph::NodeId to(opp_r, static_cast<uint32_t>(nodes_per_rover / 2));
        geometry::SE2 z_true = ds.ground_truth_poses[from].inverse() * ds.ground_truth_poses[to];
        geometry::Twist2D noise(dist_odom_t(rng) * 0.5, dist_odom_t(rng) * 0.5, dist_odom_r(rng) * 0.5);
        geometry::SE2 z_meas = z_true * geometry::SE2::exp(noise);
        add_loop_closure_helper(ds.full_graph, from, to, z_meas.to_pose2d(), lc_info, graph::EdgeVerificationStatus::Verified);
        ds.num_verified_loop_closures++;
    }

    partition_multi_rover_dataset(ds);
    return ds;
}

} // namespace astromesh::tests::support
