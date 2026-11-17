#include "astromesh/simulation/baseline_astromesh_v1.hpp"
#include "astromesh/simulation/observation_model.hpp"
#include "astromesh/simulation/pipeline_astromesh_x.hpp"
#include "astromesh/simulation/swarm_trajectories.hpp"
#include "astromesh/simulation/synthetic_world.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace astromesh;
using namespace astromesh::simulation;

struct SimParams {
    size_t rovers{2};
    std::string scenario{"ConcentricLoops"}; // ConcentricLoops, OverlappingGrid, ParallelSearch
    double duration_s{8.0};
    double step_dt_s{0.1};
    double loss_rate{0.0};
    uint64_t delay_ms{0};
    double comm_range{25.0};
    double sensor_range{10.0};
    double speed{1.0};
    double arena_extent{40.0};
    size_t num_landmarks{100};
    std::string variant{"AstroMesh_X_Full"};
    uint64_t seed{42};
    std::string output_json{""};
    std::string output_csv{""};
    int demo_id{0};
    bool run_optimize{true};
    size_t opt_steps{15};
    double partition_start_s{0.0};
    double partition_end_s{0.0};
};

static void print_usage(const char* prog) {
    std::cout << "AstroMesh-X Simulation Engine & Telemetry Generator\n"
              << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --rovers <N>          Number of rovers (default: 2, supports 1-16)\n"
              << "  --scenario <name>     Trajectory type: ConcentricLoops, OverlappingGrid, ParallelSearch (default: ConcentricLoops)\n"
              << "  --duration <sec>      Simulation duration in seconds (default: 8.0)\n"
              << "  --step-dt <sec>       Timestep delta in seconds (default: 0.1)\n"
              << "  --loss <rate>         Channel packet loss rate [0.0 - 0.9] (default: 0.0)\n"
              << "  --delay-ms <ms>       One-way link delay in milliseconds (default: 0)\n"
              << "  --comm-range <m>      Rover communication range in meters (default: 25.0)\n"
              << "  --sensor-range <m>    Landmark perception range in meters (default: 10.0)\n"
              << "  --partition-start <s> Timestamp in seconds when partition begins (default: 0.0)\n"
              << "  --partition-end <s>   Timestamp in seconds when partition heals (default: 0.0)\n"
              << "  --variant <name>      Algorithm: AstroMesh_X_Full, AstroMesh_X_RawExchange, AstroMesh_X_NoACKs,\n"
              << "                                   AstroMesh_X_NoRetries, AstroMesh_X_NoMultiHop, AstroMesh_X_Minimal,\n"
              << "                                   AstroMesh_v1_Baseline (default: AstroMesh_X_Full)\n"
              << "  --seed <uint64>       RNG seed (default: 42)\n"
              << "  --opt-steps <N>       Optimization steps (default: 15)\n"
              << "  --no-optimize         Disable graph optimization pass\n"
              << "  --demo <1|2|3|4>      Preset demonstration scenario:\n"
              << "                          1: Basic Multi-Rover SLAM (2 rovers, nominal)\n"
              << "                          2: Network Partition & Recovery (4 rovers, partition t=3-6s, 50% loss)\n"
              << "                          3: Multi-Rover Swarm (8 rovers, multi-hop mesh, 20% loss, 50 ms delay)\n"
              << "                          4: Swarm Scalability Stress Test (16 rovers, parallel search)\n"
              << "  --output-json <path>  Export detailed per-step telemetry JSON for UI visualization\n"
              << "  --output-csv <path>   Export step-by-step summary CSV\n"
              << "  --help                Display this help message\n";
}

static SimParams parse_args(int argc, char* argv[]) {
    SimParams p;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else if (arg == "--no-optimize") {
            p.run_optimize = false;
        } else if (arg == "--opt-steps" && i + 1 < argc) {
            p.opt_steps = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--rovers" && i + 1 < argc) {
            p.rovers = static_cast<size_t>(std::stoul(argv[++i]));
        } else if (arg == "--scenario" && i + 1 < argc) {
            p.scenario = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            p.duration_s = std::stod(argv[++i]);
        } else if (arg == "--step-dt" && i + 1 < argc) {
            p.step_dt_s = std::stod(argv[++i]);
        } else if (arg == "--loss" && i + 1 < argc) {
            p.loss_rate = std::stod(argv[++i]);
        } else if (arg == "--delay-ms" && i + 1 < argc) {
            p.delay_ms = std::stoull(argv[++i]);
        } else if (arg == "--comm-range" && i + 1 < argc) {
            p.comm_range = std::stod(argv[++i]);
        } else if (arg == "--sensor-range" && i + 1 < argc) {
            p.sensor_range = std::stod(argv[++i]);
        } else if (arg == "--partition-start" && i + 1 < argc) {
            p.partition_start_s = std::stod(argv[++i]);
        } else if (arg == "--partition-end" && i + 1 < argc) {
            p.partition_end_s = std::stod(argv[++i]);
        } else if (arg == "--variant" && i + 1 < argc) {
            p.variant = argv[++i];
        } else if (arg == "--seed" && i + 1 < argc) {
            p.seed = std::stoull(argv[++i]);
        } else if (arg == "--output-json" && i + 1 < argc) {
            p.output_json = argv[++i];
        } else if (arg == "--output-csv" && i + 1 < argc) {
            p.output_csv = argv[++i];
        } else if (arg == "--demo" && i + 1 < argc) {
            p.demo_id = std::stoi(argv[++i]);
        }
    }

    // Apply Demo presets if requested
    if (p.demo_id == 1) {
        // Demo 1: Basic Multi-Rover SLAM
        p.rovers = 2;
        p.scenario = "ConcentricLoops";
        p.duration_s = 8.0;
        p.loss_rate = 0.0;
        p.delay_ms = 0;
        p.comm_range = 30.0;
        p.variant = "AstroMesh_X_Full";
        p.seed = 42;
    } else if (p.demo_id == 2) {
        // Demo 2: Communication Failure & Partition Recovery
        p.rovers = 4;
        p.scenario = "OverlappingGrid";
        p.duration_s = 10.0;
        p.loss_rate = 0.50; // 50% packet drop
        p.delay_ms = 100;   // 100 ms link latency
        p.comm_range = 25.0;
        p.variant = "AstroMesh_X_Full";
        p.seed = 1042;
        p.partition_start_s = 3.0;
        p.partition_end_s = 6.0;
    } else if (p.demo_id == 3) {
        // Demo 3: Multi-Rover Swarm (8 Rovers with Multi-Hop)
        p.rovers = 8;
        p.scenario = "ConcentricLoops";
        p.duration_s = 10.0;
        p.loss_rate = 0.20; // 20% packet drop
        p.delay_ms = 50;    // 50 ms delay
        p.comm_range = 22.0;
        p.variant = "AstroMesh_X_Full";
        p.seed = 2084;
        p.opt_steps = 10;
    } else if (p.demo_id == 4) {
        // Demo 4: Swarm Scalability Stress Test (16 Rovers)
        p.rovers = 16;
        p.scenario = "ParallelSearch";
        p.duration_s = 8.0;
        p.loss_rate = 0.10;
        p.delay_ms = 50;
        p.comm_range = 25.0;
        p.variant = "AstroMesh_X_Full";
        p.seed = 4096;
        p.opt_steps = 5;
    }

    return p;
}

int main(int argc, char* argv[]) {
    SimParams p = parse_args(argc, argv);

    std::cout << "=================================================================\n"
              << "                  ASTRAMESH-X SIMULATION ENGINE                  \n"
              << "=================================================================\n"
              << "Rovers:           " << p.rovers << "\n"
              << "Scenario:         " << p.scenario << "\n"
              << "Duration:         " << p.duration_s << " s (dt=" << p.step_dt_s << " s)\n"
              << "Packet Loss:      " << (p.loss_rate * 100.0) << " %\n"
              << "One-Way Delay:    " << p.delay_ms << " ms\n"
              << "Comm Range:       " << p.comm_range << " m\n"
              << "Variant:          " << p.variant << "\n"
              << "Seed:             " << p.seed << "\n";
    if (p.demo_id > 0) {
        std::cout << "Demo Preset:      Demo " << p.demo_id << "\n";
    }
    std::cout << "-----------------------------------------------------------------\n";

    const uint64_t duration_ns = static_cast<uint64_t>(p.duration_s * 1e9);
    const uint64_t step_dt_ns = static_cast<uint64_t>(p.step_dt_s * 1e9);
    const size_t num_steps = static_cast<size_t>(duration_ns / step_dt_ns) + 1;

    // 1. Synthetic World Generation
    WorldConfig wcfg;
    wcfg.num_landmarks = p.num_landmarks;
    wcfg.sensor_range = p.sensor_range;
    wcfg.seed = p.seed ^ 0x9e3779b97f4a7c15ULL;
    wcfg.min_x = -p.arena_extent;
    wcfg.max_x = p.arena_extent;
    wcfg.min_y = -p.arena_extent;
    wcfg.max_y = p.arena_extent;
    SyntheticWorld2D world(wcfg);

    // 2. Trajectory Generation
    TrajectoryConfig tcfg;
    tcfg.num_rovers = p.rovers;
    tcfg.duration_ns = duration_ns;
    tcfg.step_dt_ns = step_dt_ns;
    tcfg.speed = p.speed;
    tcfg.arena_extent = p.arena_extent;
    tcfg.seed = p.seed ^ 0x517cc1b727220a95ULL;

    std::vector<RoverTrajectory> trajectories;
    if (p.scenario == "ConcentricLoops") {
        trajectories = SwarmTrajectoryGenerator::generate_concentric_loops(tcfg);
    } else if (p.scenario == "OverlappingGrid") {
        trajectories = SwarmTrajectoryGenerator::generate_overlapping_grid(tcfg);
    } else {
        trajectories = SwarmTrajectoryGenerator::generate_parallel_search(tcfg);
    }

    // 3. Sensor Observation Model
    ObservationNoiseConfig obs_noise;
    obs_noise.range_std_dev = 0.05;
    obs_noise.bearing_std_dev = 0.01;
    obs_noise.seed = p.seed ^ 0x3141592653589793ULL;
    ObservationModel2D obs_model(obs_noise);

    // 4. Construct Pipeline Variant
    std::unique_ptr<IRoverSwarmPipeline> pipeline;
    const uint64_t delay_ns = p.delay_ms * 1'000'000ULL;

    auto make_base_x_config = [&]() {
        AstroMeshXConfig c;
        c.seed = p.seed;
        c.communication_range = p.comm_range;
        c.odometry_noise_trans = 0.02;
        c.odometry_noise_rot = 0.005;
        c.default_one_way_delay_ns = delay_ns;
        c.packet_loss_rate = p.loss_rate;
        c.simulation_duration_ns = duration_ns;
        c.enable_cord = p.run_optimize;
        c.cord_options.max_steps = p.opt_steps; // Configurable for real-time responsiveness
        return c;
    };

    if (p.variant == "AstroMesh_X_Full") {
        auto cfg = make_base_x_config();
        pipeline = std::make_unique<AstroMeshXPipeline>(cfg);
    } else if (p.variant == "AstroMesh_X_RawExchange") {
        auto cfg = make_base_x_config();
        cfg.discovery_mechanism = AstroMeshXConfig::DiscoveryMechanism::RawExchange;
        cfg.custom_pipeline_name = "AstroMesh_X_RawExchange";
        pipeline = std::make_unique<AstroMeshXPipeline>(cfg);
    } else if (p.variant == "AstroMesh_X_NoACKs") {
        auto cfg = make_base_x_config();
        cfg.enable_explicit_acks = false;
        cfg.custom_pipeline_name = "AstroMesh_X_NoACKs";
        pipeline = std::make_unique<AstroMeshXPipeline>(cfg);
    } else if (p.variant == "AstroMesh_X_NoRetries") {
        auto cfg = make_base_x_config();
        cfg.enable_dtn_retries = false;
        cfg.custom_pipeline_name = "AstroMesh_X_NoRetries";
        pipeline = std::make_unique<AstroMeshXPipeline>(cfg);
    } else if (p.variant == "AstroMesh_X_NoMultiHop") {
        auto cfg = make_base_x_config();
        cfg.enable_multihop_routing = false;
        cfg.custom_pipeline_name = "AstroMesh_X_NoMultiHop";
        pipeline = std::make_unique<AstroMeshXPipeline>(cfg);
    } else if (p.variant == "AstroMesh_X_Minimal") {
        auto cfg = make_base_x_config();
        cfg.enable_explicit_acks = false;
        cfg.enable_dtn_retries = false;
        cfg.enable_multihop_routing = false;
        cfg.custom_pipeline_name = "AstroMesh_X_Minimal";
        pipeline = std::make_unique<AstroMeshXPipeline>(cfg);
    } else {
        // Baseline v1
        BaselineV1Config bcfg;
        bcfg.seed = p.seed;
        bcfg.communication_range = p.comm_range;
        bcfg.odometry_noise_trans = 0.02;
        bcfg.odometry_noise_rot = 0.005;
        bcfg.default_one_way_delay_ns = delay_ns;
        bcfg.simulation_duration_ns = duration_ns;
        pipeline = std::make_unique<BaselineAstroMeshV1>(bcfg);
    }

    std::cout << "Starting simulation execution (" << num_steps << " timesteps)...\n";
    const auto sim_start_time = std::chrono::high_resolution_clock::now();

    // Data structures for JSON telemetry export
    struct StepTelemetry {
        size_t step_index{0};
        double timestamp_s{0.0};
        struct RoverTelemetry {
            uint16_t rover_id{0};
            double gt_x{0.0};
            double gt_y{0.0};
            double gt_theta{0.0};
            double est_x{0.0};
            double est_y{0.0};
            double est_theta{0.0};
            size_t obs_count{0};
            std::vector<uint32_t> landmark_ids;
        };
        std::vector<RoverTelemetry> rovers;

        struct LinkTelemetry {
            uint16_t rover_a{0};
            uint16_t rover_b{0};
            double distance{0.0};
            bool in_range{false};
            bool active{false};
            bool partitioned{false};
        };
        std::vector<LinkTelemetry> links;

        size_t components{0};
        size_t verified_closures_total{0};
        size_t total_wire_bytes{0};
        size_t logical_messages_sent{0};
        size_t logical_messages_delivered{0};
        size_t logical_messages_dropped{0};
        bool is_partitioned{false};
    };

    std::vector<StepTelemetry> step_records;
    step_records.reserve(num_steps);

    size_t prev_closures_count = 0;

    for (size_t s = 0; s < num_steps; ++s) {
        SimulationStepInput step;
        step.timestamp_ns = s * step_dt_ns;
        const double time_s = static_cast<double>(step.timestamp_ns) * 1e-9;

        StepTelemetry step_telem;
        step_telem.step_index = s;
        step_telem.timestamp_s = time_s;

        const bool partition_active = (p.partition_end_s > p.partition_start_s &&
                                       time_s >= p.partition_start_s &&
                                       time_s < p.partition_end_s);
        step_telem.is_partitioned = partition_active;

        // Ingest Rover Steps
        for (size_t r = 0; r < p.rovers; ++r) {
            RoverStepData rdata;
            rdata.rover_id = static_cast<uint16_t>(r + 1);
            rdata.timestamp_ns = step.timestamp_ns;

            const auto& curr_gt = trajectories[r].waypoints[s].pose;
            if (s == 0) {
                rdata.odometry_step = geometry::SE2::identity();
            } else {
                const auto& prev_gt = trajectories[r].waypoints[s - 1].pose;
                rdata.odometry_step = prev_gt.inverse().compose(curr_gt);
            }

            rdata.observations = obs_model.observe(world, curr_gt, rdata.rover_id);
            step.rover_steps.push_back(rdata);

            StepTelemetry::RoverTelemetry rt;
            rt.rover_id = rdata.rover_id;
            rt.gt_x = curr_gt.x();
            rt.gt_y = curr_gt.y();
            rt.gt_theta = curr_gt.theta();
            rt.obs_count = rdata.observations.size();
            for (const auto& obs : rdata.observations) {
                rt.landmark_ids.push_back(obs.landmark_id);
            }
            step_telem.rovers.push_back(rt);
        }

        // Compute Pairwise Links
        for (size_t i = 0; i < p.rovers; ++i) {
            for (size_t j = i + 1; j < p.rovers; ++j) {
                const auto& pose_a = trajectories[i].waypoints[s].pose;
                const auto& pose_b = trajectories[j].waypoints[s].pose;
                const double dist = std::hypot(pose_a.x() - pose_b.x(), pose_a.y() - pose_b.y());

                // If partition is active, sever links between first half and second half
                const bool cross_partition = partition_active && (i < (p.rovers / 2) && j >= (p.rovers / 2));

                ChannelLinkState link;
                link.rover_a = static_cast<uint16_t>(i + 1);
                link.rover_b = static_cast<uint16_t>(j + 1);
                link.in_communication_range = (dist <= p.comm_range) && !cross_partition;
                link.is_partitioned = cross_partition;
                link.link_packet_loss_rate = cross_partition ? 1.0 : p.loss_rate;
                link.one_way_delay_ns = delay_ns;
                step.channel_states.push_back(link);

                StepTelemetry::LinkTelemetry lt;
                lt.rover_a = link.rover_a;
                lt.rover_b = link.rover_b;
                lt.distance = dist;
                lt.in_range = link.in_communication_range;
                lt.active = link.in_communication_range && (p.loss_rate < 1.0);
                lt.partitioned = cross_partition;
                step_telem.links.push_back(lt);
            }
        }

        // Ingest step into pipeline
        const bool ok = pipeline->ingest_step(step);
        if (!ok) {
            std::cerr << "ERROR: Pipeline step " << s << " rejected input!\n";
            return 1;
        }

        // Populate estimated poses
        for (size_t r = 0; r < p.rovers; ++r) {
            auto est_pose = pipeline->get_current_pose(static_cast<uint16_t>(r + 1));
            if (est_pose.has_value()) {
                step_telem.rovers[r].est_x = est_pose->x();
                step_telem.rovers[r].est_y = est_pose->y();
                step_telem.rovers[r].est_theta = est_pose->theta();
            }
        }

        const auto m = pipeline->collect_metrics();
        step_telem.components = pipeline->num_connected_components();
        step_telem.verified_closures_total = m.loop_closures_verified;
        step_telem.total_wire_bytes = m.total_wire_bytes_transmitted;
        step_telem.logical_messages_sent = m.logical_messages_sent;
        step_telem.logical_messages_delivered = m.logical_messages_delivered;
        step_telem.logical_messages_dropped = m.logical_messages_dropped;

        if (m.loop_closures_verified > prev_closures_count) {
            std::cout << "  [t=" << std::fixed << std::setprecision(1) << time_s << "s] Loop Closure Verified! "
                      << "Total verified: " << m.loop_closures_verified
                      << " | DSU Components: " << step_telem.components << "\n";
            prev_closures_count = m.loop_closures_verified;
        }

        step_records.push_back(step_telem);
    }

    // Run Optimization
    bool opt_ok = false;
    if (p.run_optimize) {
        std::cout << "Executing graph optimization (" << p.opt_steps << " steps)...\n";
        opt_ok = pipeline->optimize();
    } else {
        std::cout << "Graph optimization pass skipped (--no-optimize).\n";
    }
    const auto sim_end_time = std::chrono::high_resolution_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(sim_end_time - sim_start_time).count();

    const auto final_metrics = pipeline->collect_metrics();
    const auto loop_history = pipeline->get_loop_closure_history();
    const auto est_trajectories = pipeline->get_estimated_trajectories();

    // Compute Trajectory ATE RMSE
    double ate_sum_sq = 0.0;
    size_t ate_samples = 0;
    for (size_t r = 0; r < p.rovers; ++r) {
        uint16_t rid = static_cast<uint16_t>(r + 1);
        auto it = est_trajectories.find(rid);
        if (it != est_trajectories.end()) {
            const auto& est_vec = it->second;
            const auto& gt_vec = trajectories[r].waypoints;
            const size_t n = std::min(est_vec.size(), gt_vec.size());
            for (size_t i = 0; i < n; ++i) {
                double dx = est_vec[i].x() - gt_vec[i].pose.x();
                double dy = est_vec[i].y() - gt_vec[i].pose.y();
                ate_sum_sq += (dx * dx + dy * dy);
                ate_samples++;
            }
        }
    }
    const double ate_rmse = (ate_samples > 0) ? std::sqrt(ate_sum_sq / ate_samples) : 0.0;

    std::cout << "\n=================================================================\n"
              << "                   SIMULATION RUN RESULTS                        \n"
              << "=================================================================\n"
              << "Elapsed Wall Time:          " << std::fixed << std::setprecision(2) << elapsed_ms << " ms\n"
              << "Optimization Completed:     " << (opt_ok ? "True" : "False/Unsupported") << "\n"
              << "Initial DSU Components:     " << p.rovers << "\n"
              << "Final DSU Components:       " << pipeline->num_connected_components() << "\n"
              << "Loop Closures Verified:     " << final_metrics.loop_closures_verified << "\n"
              << "Loop Closures Rejected:     " << final_metrics.loop_closures_rejected << "\n"
              << "Application Delivery Ratio: " << std::setprecision(2) << (final_metrics.application_completion_ratio * 100.0) << " %\n"
              << "Per-Hop Delivery Ratio:     " << (final_metrics.per_hop_delivery_ratio * 100.0) << " %\n"
              << "Logical Messages Sent:      " << final_metrics.logical_messages_sent << "\n"
              << "Logical Messages Delivered: " << final_metrics.logical_messages_delivered << "\n"
              << "Logical Messages Dropped:   " << final_metrics.logical_messages_dropped << "\n"
              << "Retransmissions Performed:  " << final_metrics.retransmissions_performed << "\n"
              << "Payload Wire Bytes:         " << final_metrics.payload_bytes_transmitted << " B\n"
              << "Protocol Overhead Bytes:    " << final_metrics.protocol_bytes_transmitted << " B\n"
              << "Framing Overhead Bytes:     " << final_metrics.framing_bytes_transmitted << " B\n"
              << "ACK Wire Bytes:             " << final_metrics.ack_wire_bytes_transmitted << " B\n"
              << "Total Wire Bytes:           " << final_metrics.total_wire_bytes_transmitted << " B\n"
              << "Trajectory ATE RMSE:        " << std::setprecision(4) << ate_rmse << " m\n"
              << "=================================================================\n";

    // 5. Export JSON Telemetry if requested
    if (!p.output_json.empty()) {
        std::cout << "Writing structured JSON telemetry to " << p.output_json << " ...\n";
        std::ofstream jout(p.output_json);
        if (!jout.is_open()) {
            std::cerr << "Failed to open " << p.output_json << " for writing!\n";
            return 1;
        }

        jout << "{\n";
        jout << "  \"config\": {\n"
             << "    \"rovers\": " << p.rovers << ",\n"
             << "    \"scenario\": \"" << p.scenario << "\",\n"
             << "    \"duration_s\": " << p.duration_s << ",\n"
             << "    \"step_dt_s\": " << p.step_dt_s << ",\n"
             << "    \"loss_rate\": " << p.loss_rate << ",\n"
             << "    \"delay_ms\": " << p.delay_ms << ",\n"
             << "    \"comm_range\": " << p.comm_range << ",\n"
             << "    \"sensor_range\": " << p.sensor_range << ",\n"
             << "    \"variant\": \"" << p.variant << "\",\n"
             << "    \"seed\": " << p.seed << ",\n"
             << "    \"arena_extent\": " << p.arena_extent << "\n"
             << "  },\n";

        // Summary
        jout << "  \"summary\": {\n"
             << "    \"elapsed_wall_ms\": " << elapsed_ms << ",\n"
             << "    \"optimization_ok\": " << (opt_ok ? "true" : "false") << ",\n"
             << "    \"initial_components\": " << p.rovers << ",\n"
             << "    \"final_components\": " << pipeline->num_connected_components() << ",\n"
             << "    \"loop_closures_verified\": " << final_metrics.loop_closures_verified << ",\n"
             << "    \"loop_closures_rejected\": " << final_metrics.loop_closures_rejected << ",\n"
             << "    \"ate_rmse_m\": " << ate_rmse << ",\n"
             << "    \"app_completion_ratio\": " << final_metrics.application_completion_ratio << ",\n"
             << "    \"per_hop_delivery_ratio\": " << final_metrics.per_hop_delivery_ratio << ",\n"
             << "    \"logical_messages_sent\": " << final_metrics.logical_messages_sent << ",\n"
             << "    \"logical_messages_delivered\": " << final_metrics.logical_messages_delivered << ",\n"
             << "    \"logical_messages_dropped\": " << final_metrics.logical_messages_dropped << ",\n"
             << "    \"retransmissions\": " << final_metrics.retransmissions_performed << ",\n"
             << "    \"payload_bytes\": " << final_metrics.payload_bytes_transmitted << ",\n"
             << "    \"protocol_bytes\": " << final_metrics.protocol_bytes_transmitted << ",\n"
             << "    \"framing_bytes\": " << final_metrics.framing_bytes_transmitted << ",\n"
             << "    \"ack_wire_bytes\": " << final_metrics.ack_wire_bytes_transmitted << ",\n"
             << "    \"total_wire_bytes\": " << final_metrics.total_wire_bytes_transmitted << "\n"
             << "  },\n";

        // Landmarks
        jout << "  \"landmarks\": [\n";
        for (size_t i = 0; i < world.landmarks().size(); ++i) {
            const auto& lm = world.landmarks()[i];
            jout << "    {\"id\": " << lm.id << ", \"x\": " << std::fixed << std::setprecision(3) << lm.position.x
                 << ", \"y\": " << lm.position.y << "}" << (i + 1 < world.landmarks().size() ? "," : "") << "\n";
        }
        jout << "  ],\n";

        // Loop Closures History
        jout << "  \"loop_closure_events\": [\n";
        for (size_t i = 0; i < loop_history.size(); ++i) {
            const auto& c = loop_history[i];
            jout << "    {\"rover_a\": " << c.rover_a << ", \"rover_b\": " << c.rover_b
                 << ", \"timestamp_ns\": " << c.timestamp_ns
                 << ", \"matched_landmarks\": " << c.candidate_landmarks_matched
                 << ", \"passed\": " << (c.verification_passed ? "true" : "false")
                 << ", \"inliers\": " << c.inlier_count
                 << ", \"rmse\": " << std::setprecision(4) << c.inlier_rmse
                 << "}" << (i + 1 < loop_history.size() ? "," : "") << "\n";
        }
        jout << "  ],\n";

        // Step by Step Telemetry Frames
        jout << "  \"frames\": [\n";
        for (size_t s = 0; s < step_records.size(); ++s) {
            const auto& f = step_records[s];
            jout << "    {\n"
                 << "      \"step\": " << f.step_index << ",\n"
                 << "      \"time_s\": " << std::fixed << std::setprecision(2) << f.timestamp_s << ",\n"
                 << "      \"components\": " << f.components << ",\n"
                 << "      \"verified_closures\": " << f.verified_closures_total << ",\n"
                 << "      \"wire_bytes\": " << f.total_wire_bytes << ",\n"
                 << "      \"rovers\": [\n";
            for (size_t r = 0; r < f.rovers.size(); ++r) {
                const auto& rv = f.rovers[r];
                jout << "        {\"id\": " << rv.rover_id
                     << ", \"gt\": [" << std::setprecision(3) << rv.gt_x << ", " << rv.gt_y << ", " << rv.gt_theta << "]"
                     << ", \"est\": [" << rv.est_x << ", " << rv.est_y << ", " << rv.est_theta << "]"
                     << ", \"obs\": " << rv.obs_count << "}"
                     << (r + 1 < f.rovers.size() ? "," : "") << "\n";
            }
            jout << "      ],\n"
                 << "      \"links\": [\n";
            for (size_t l = 0; l < f.links.size(); ++l) {
                const auto& lk = f.links[l];
                jout << "        {\"a\": " << lk.rover_a << ", \"b\": " << lk.rover_b
                     << ", \"dist\": " << std::setprecision(2) << lk.distance
                     << ", \"in_range\": " << (lk.in_range ? "true" : "false")
                     << ", \"active\": " << (lk.active ? "true" : "false")
                     << ", \"partitioned\": " << (lk.partitioned ? "true" : "false") << "}"
                     << (l + 1 < f.links.size() ? "," : "") << "\n";
            }
            jout << "      ],\n"
                 << "      \"is_partitioned\": " << (f.is_partitioned ? "true" : "false") << "\n"
                 << "    }" << (s + 1 < step_records.size() ? "," : "") << "\n";
        }
        jout << "  ]\n";
        jout << "}\n";
        jout.close();
        std::cout << "JSON telemetry export complete!\n";
    }

    // 6. Export CSV if requested
    if (!p.output_csv.empty()) {
        std::cout << "Writing summary CSV to " << p.output_csv << " ...\n";
        std::ofstream cout(p.output_csv);
        if (cout.is_open()) {
            cout << "step,time_s,components,verified_closures,total_wire_bytes,messages_sent,messages_delivered,messages_dropped\n";
            for (const auto& f : step_records) {
                cout << f.step_index << ","
                     << std::fixed << std::setprecision(2) << f.timestamp_s << ","
                     << f.components << ","
                     << f.verified_closures_total << ","
                     << f.total_wire_bytes << ","
                     << f.logical_messages_sent << ","
                     << f.logical_messages_delivered << ","
                     << f.logical_messages_dropped << "\n";
            }
            cout.close();
        }
    }

    std::cout << "Simulation completed successfully.\n";
    return 0;
}
