#include "astromesh/graph/keyframe_pruner.hpp"
#include "astromesh/graph/pose_graph.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace astromesh;
using namespace astromesh::graph;

struct BenchmarkScenario {
    std::string name;
    std::string topology; // Chain, Loop, Grid, Disconnected
    size_t target_nodes{0};
    size_t max_retained_bound{0};
    double loop_density{0.0}; // Fraction of nodes with loop closures
};

struct BenchmarkRecord {
    std::string scenario_name;
    std::string topology;
    size_t max_retained_bound{0};
    size_t initial_nodes{0};
    size_t final_nodes{0};
    size_t initial_edges{0};
    size_t final_edges{0};
    size_t nodes_pruned{0};
    size_t edges_pruned{0};
    size_t protected_nodes{0};
    std::string bound_status;
    size_t initial_components{0};
    size_t final_components{0};
    size_t verified_loops_initial{0};
    size_t verified_loops_final{0};
    size_t struct_bytes_initial{0};
    size_t struct_bytes_final{0};
    size_t serialized_bytes_initial{0};
    size_t serialized_bytes_final{0};
    double memory_reduction_pct{0.0};
};

PoseGraph build_synthetic_topology(const BenchmarkScenario& sc) {
    PoseGraph g;
    uint64_t next_edge_id = 1;

    if (sc.topology == "Chain") {
        for (uint32_t i = 0; i < sc.target_nodes; ++i) {
            KeyframeNode kf;
            kf.id = NodeId(1, i);
            kf.pose = Pose2D(static_cast<double>(i) * 0.5, 0.0, 0.0);
            kf.observed_landmarks.push_back(core::LandmarkKey(1, 1, i, 0x1000 + i));
            g.add_node(kf);
            if (i > 0) {
                PoseGraphEdge e(next_edge_id++, NodeId(1, i - 1), NodeId(1, i),
                                EdgeType::Odometry, Pose2D(0.5, 0.0, 0.0),
                                InformationMatrix2D::diagonal(100.0, 1000.0),
                                EdgeVerificationStatus::Verified);
                g.add_edge(e);
            }
        }
    } else if (sc.topology == "Loop") {
        for (uint32_t i = 0; i < sc.target_nodes; ++i) {
            const double angle = 2.0 * 3.141592653589793 * static_cast<double>(i) / static_cast<double>(sc.target_nodes);
            const double radius = 10.0;
            KeyframeNode kf;
            kf.id = NodeId(1, i);
            kf.pose = Pose2D(radius * std::cos(angle), radius * std::sin(angle), angle);
            kf.observed_landmarks.push_back(core::LandmarkKey(1, 1, i, 0x2000 + i));
            g.add_node(kf);
            if (i > 0) {
                PoseGraphEdge e(next_edge_id++, NodeId(1, i - 1), NodeId(1, i),
                                EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0),
                                InformationMatrix2D::diagonal(100.0, 1000.0),
                                EdgeVerificationStatus::Verified);
                g.add_edge(e);
            }
        }
        // Closing loop edge between target_nodes - 1 and 0
        if (sc.target_nodes > 2) {
            PoseGraphEdge closing(next_edge_id++, NodeId(1, sc.target_nodes - 1), NodeId(1, 0),
                                 EdgeType::IntraRoverLoopClosure, Pose2D(1.0, 0.0, 0.0),
                                 InformationMatrix2D::diagonal(100.0, 1000.0),
                                 EdgeVerificationStatus::Verified);
            g.add_edge(closing);
        }
        // Additional chords if loop_density > 0
        if (sc.loop_density > 0.0 && sc.target_nodes >= 6) {
            size_t num_chords = static_cast<size_t>(sc.target_nodes * sc.loop_density);
            for (size_t c = 1; c <= num_chords; ++c) {
                uint32_t from = static_cast<uint32_t>(c);
                uint32_t to = static_cast<uint32_t>(sc.target_nodes - c - 1);
                if (from < to) {
                    PoseGraphEdge chord(next_edge_id++, NodeId(1, from), NodeId(1, to),
                                       EdgeType::IntraRoverLoopClosure, Pose2D(0.0, 0.0, 0.0),
                                       InformationMatrix2D::diagonal(50.0, 500.0),
                                       EdgeVerificationStatus::Verified);
                    g.add_edge(chord);
                }
            }
        }
    } else if (sc.topology == "Grid") {
        // 2D lattice: size K x K
        size_t K = static_cast<size_t>(std::sqrt(sc.target_nodes));
        if (K * K < sc.target_nodes) K++;
        uint32_t idx = 0;
        for (size_t r = 0; r < K && idx < sc.target_nodes; ++r) {
            for (size_t c = 0; c < K && idx < sc.target_nodes; ++c) {
                KeyframeNode kf;
                kf.id = NodeId(1, idx);
                kf.pose = Pose2D(static_cast<double>(c) * 1.0, static_cast<double>(r) * 1.0, 0.0);
                kf.observed_landmarks.push_back(core::LandmarkKey(1, 1, idx, 0x3000 + idx));
                g.add_node(kf);
                // Connect to left neighbor
                if (c > 0 && idx > 0) {
                    PoseGraphEdge e(next_edge_id++, NodeId(1, idx - 1), NodeId(1, idx),
                                    EdgeType::Odometry, Pose2D(1.0, 0.0, 0.0),
                                    InformationMatrix2D::diagonal(100.0, 1000.0),
                                    EdgeVerificationStatus::Verified);
                    g.add_edge(e);
                }
                // Connect to neighbor in row above
                if (r > 0 && idx >= K) {
                    PoseGraphEdge e(next_edge_id++, NodeId(1, idx - K), NodeId(1, idx),
                                    EdgeType::IntraRoverLoopClosure, Pose2D(0.0, 1.0, 0.0),
                                    InformationMatrix2D::diagonal(100.0, 1000.0),
                                    EdgeVerificationStatus::Verified);
                    g.add_edge(e);
                }
                idx++;
            }
        }
    } else if (sc.topology == "Disconnected") {
        // Two disjoint subgraphs: Rover 1 chain and Rover 2 chain
        size_t half = sc.target_nodes / 2;
        for (uint32_t i = 0; i < half; ++i) {
            KeyframeNode kf;
            kf.id = NodeId(1, i);
            kf.pose = Pose2D(static_cast<double>(i) * 0.5, 0.0, 0.0);
            g.add_node(kf);
            if (i > 0) {
                g.add_edge(PoseGraphEdge(next_edge_id++, NodeId(1, i - 1), NodeId(1, i),
                           EdgeType::Odometry, Pose2D(0.5, 0.0, 0.0),
                           InformationMatrix2D::diagonal(100.0, 1000.0), EdgeVerificationStatus::Verified));
            }
        }
        for (uint32_t i = 0; i < (sc.target_nodes - half); ++i) {
            KeyframeNode kf;
            kf.id = NodeId(2, i);
            kf.pose = Pose2D(10.0 + static_cast<double>(i) * 0.5, 0.0, 0.0);
            g.add_node(kf);
            if (i > 0) {
                g.add_edge(PoseGraphEdge(next_edge_id++, NodeId(2, i - 1), NodeId(2, i),
                           EdgeType::Odometry, Pose2D(0.5, 0.0, 0.0),
                           InformationMatrix2D::diagonal(100.0, 1000.0), EdgeVerificationStatus::Verified));
            }
        }
    }

    return g;
}

BenchmarkRecord run_scenario(const BenchmarkScenario& sc) {
    PoseGraph g = build_synthetic_topology(sc);
    SubgraphPartitionManager dsu(g);

    PrunerConfig cfg;
    cfg.max_retained_keyframes = sc.max_retained_bound;
    cfg.protect_gauge_anchors = true;
    cfg.protect_newest_keyframe = true;
    cfg.protect_verified_loop_endpoints = true;
    cfg.protect_inter_rover_endpoints = true;
    cfg.protect_articulation_points = true;

    KeyframePruner pruner(cfg);

    const auto mem_init = pruner.compute_memory_metrics(g);
    const size_t comp_init = dsu.num_partitions();

    PruningResult res = pruner.enforce_memory_bound(g, &dsu);

    const auto mem_final = pruner.compute_memory_metrics(g);
    const size_t comp_final = dsu.num_partitions();

    BenchmarkRecord r;
    r.scenario_name = sc.name;
    r.topology = sc.topology;
    r.max_retained_bound = sc.max_retained_bound;
    r.initial_nodes = mem_init.node_count;
    r.final_nodes = mem_final.node_count;
    r.initial_edges = mem_init.edge_count;
    r.final_edges = mem_final.edge_count;
    r.nodes_pruned = res.nodes_pruned;
    r.edges_pruned = res.edges_pruned;
    r.protected_nodes = res.protected_nodes;
    r.bound_status = to_string(res.status);
    r.initial_components = comp_init;
    r.final_components = comp_final;
    r.verified_loops_initial = mem_init.verified_loop_closure_count;
    r.verified_loops_final = mem_final.verified_loop_closure_count;
    r.struct_bytes_initial = mem_init.estimated_struct_bytes;
    r.struct_bytes_final = mem_final.estimated_struct_bytes;
    r.serialized_bytes_initial = mem_init.estimated_serialized_bytes;
    r.serialized_bytes_final = mem_final.estimated_serialized_bytes;

    r.memory_reduction_pct = (r.struct_bytes_initial > 0) ?
        ((1.0 - static_cast<double>(r.struct_bytes_final) / static_cast<double>(r.struct_bytes_initial)) * 100.0) : 0.0;

    return r;
}

void write_csv_header(std::ostream& os) {
    os << "scenario_name,topology,max_retained_bound,initial_nodes,final_nodes,initial_edges,"
       << "final_edges,nodes_pruned,edges_pruned,protected_nodes,bound_status,initial_components,"
       << "final_components,verified_loops_initial,verified_loops_final,struct_bytes_initial,"
       << "struct_bytes_final,serialized_bytes_initial,serialized_bytes_final,memory_reduction_pct\n";
}

void write_csv_row(std::ostream& os, const BenchmarkRecord& r) {
    os << r.scenario_name << ","
       << r.topology << ","
       << r.max_retained_bound << ","
       << r.initial_nodes << ","
       << r.final_nodes << ","
       << r.initial_edges << ","
       << r.final_edges << ","
       << r.nodes_pruned << ","
       << r.edges_pruned << ","
       << r.protected_nodes << ","
       << r.bound_status << ","
       << r.initial_components << ","
       << r.final_components << ","
       << r.verified_loops_initial << ","
       << r.verified_loops_final << ","
       << r.struct_bytes_initial << ","
       << r.struct_bytes_final << ","
       << r.serialized_bytes_initial << ","
       << r.serialized_bytes_final << ","
       << std::fixed << std::setprecision(2) << r.memory_reduction_pct << "\n";
}

int main(int argc, char* argv[]) {
    std::string output_csv = "benchmark_keyframe_pruner_results.csv";
    if (argc > 1) {
        output_csv = argv[1];
    }

    std::cout << "Running AstroMesh-X Phase 4.3 Safe Keyframe Pruning Benchmark..." << std::endl;
    std::cout << "Output CSV: " << output_csv << std::endl;

    std::vector<BenchmarkScenario> scenarios;

    // 1. Baseline Unbounded (No Pruning)
    scenarios.push_back({"Baseline_Chain_Unbounded", "Chain", 30, 0, 0.0});
    scenarios.push_back({"Baseline_Loop_Unbounded", "Loop", 30, 0, 0.0});

    // 2. Chain Topology (All intermediate nodes are articulation points -> LimitUnmetNoSafeCandidate)
    scenarios.push_back({"Chain_30N_Bound20", "Chain", 30, 20, 0.0});
    scenarios.push_back({"Chain_30N_Bound15", "Chain", 30, 15, 0.0});

    // 3. Loop Topology with Chords (Safe redundant keyframe pruning enabled)
    scenarios.push_back({"Loop_30N_Density0_Bound20", "Loop", 30, 20, 0.0});
    scenarios.push_back({"Loop_30N_Density0_Bound15", "Loop", 30, 15, 0.0});
    scenarios.push_back({"Loop_30N_Density20_Bound20", "Loop", 30, 20, 0.2});
    scenarios.push_back({"Loop_30N_Density40_Bound15", "Loop", 30, 15, 0.4});

    // 4. Grid Lattice Topology (Multiple redundant paths)
    scenarios.push_back({"Grid_25N_Bound20", "Grid", 25, 20, 0.0});
    scenarios.push_back({"Grid_25N_Bound15", "Grid", 25, 15, 0.0});
    scenarios.push_back({"Grid_36N_Bound25", "Grid", 36, 25, 0.0});

    // 5. Disconnected Multi-Rover Topology
    scenarios.push_back({"Disconnected_20N_Bound15", "Disconnected", 20, 15, 0.0});
    scenarios.push_back({"Disconnected_40N_Bound25", "Disconnected", 40, 25, 0.0});

    std::ofstream csv(output_csv);
    write_csv_header(csv);

    size_t count = 0;
    for (const auto& sc : scenarios) {
        count++;
        BenchmarkRecord r = run_scenario(sc);
        write_csv_row(csv, r);
        std::cout << "[" << std::setw(2) << count << "/" << scenarios.size() << "] "
                  << std::setw(32) << std::left << r.scenario_name
                  << " Topo=" << std::setw(12) << r.topology
                  << " Nodes: " << std::setw(2) << r.initial_nodes << " -> " << std::setw(2) << r.final_nodes
                  << " Pruned=" << std::setw(2) << r.nodes_pruned
                  << " Prot=" << std::setw(2) << r.protected_nodes
                  << " Status=" << std::setw(26) << r.bound_status
                  << " MemRed=" << std::setw(5) << std::fixed << std::setprecision(1) << r.memory_reduction_pct << "%"
                  << std::endl;
    }

    csv.close();
    std::cout << "\nBenchmark complete. " << scenarios.size() << " trials written to " << output_csv << std::endl;
    return 0;
}
