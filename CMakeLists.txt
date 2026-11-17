cmake_minimum_required(VERSION 3.20)

project(AstroMeshX
    VERSION 0.1.0
    DESCRIPTION "Decentralized Collaborative Topological SLAM for Multi-Rover Exploration"
    LANGUAGES CXX
)

# Enforce standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Statically link GCC and C++ runtimes on MinGW/UCRT so test/benchmark binaries are self-contained
if(MINGW)
    add_link_options(-static)
endif()

# Ensure assertions are active in tests and benchmarks across all build types
add_compile_options(-UNDEBUG)

# Enable testing framework
include(CTest)

# Set build output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Public include directory
set(ASTROMESH_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

message(STATUS "Configuring AstroMesh-X v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Testing: ${BUILD_TESTING}")

# ==============================================================================
# Phase 1: Core Data Structures
# ==============================================================================
# Landmark Keys and Core Data Structures
add_library(astromesh_core STATIC
    src/core/landmark_key.cpp
    src/core/counting_bloom_filter.cpp
    src/core/iblt.cpp
    src/core/version_vector.cpp
)

target_include_directories(astromesh_core PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

# ==============================================================================
# Phase 2: Topological Graph Representation & Verification
# ==============================================================================
add_library(astromesh_graph STATIC
    src/graph/pose_graph.cpp
    src/graph/disjoint_set.cpp
    src/graph/keyframe_pruner.cpp
    src/graph/map_update.cpp
    src/graph/conflict_graph_merger.cpp
)

target_include_directories(astromesh_graph PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_graph PUBLIC
    astromesh_core
)

add_library(astromesh_verification STATIC
    src/verification/geometric_verifier.cpp
)

target_include_directories(astromesh_verification PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_verification PUBLIC
    astromesh_graph
)

add_library(astromesh_geometry STATIC
    src/geometry/se2.cpp
    src/geometry/subgraph_aligner.cpp
    src/graph/alignment_graph.cpp
)

target_include_directories(astromesh_geometry PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_geometry PUBLIC
    astromesh_verification
    astromesh_graph
    astromesh_core
)

add_library(astromesh_optimization STATIC
    src/optimization/pose_graph_objective.cpp
    src/optimization/trajectory_metrics.cpp
    src/optimization/centralized_optimizer.cpp
    src/optimization/cord_integrator.cpp
    src/optimization/boundary_packet.cpp
    src/optimization/simulated_mesh_network.cpp
    src/optimization/distributed_cord_node.cpp
    src/optimization/boundary_filter.cpp
)

target_include_directories(astromesh_optimization PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_optimization PUBLIC
    astromesh_geometry
    astromesh_graph
    astromesh_core
)

# ==============================================================================
# Phase 4: Delay-Tolerant Networking & Communication Layer
# ==============================================================================
add_library(astromesh_comms STATIC
    src/comms/packet_budget.cpp
    src/comms/dtn_buffer.cpp
)

target_include_directories(astromesh_comms PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_comms PUBLIC
    astromesh_core
)

# ==============================================================================
# Phase 4.4: Dynamic Multi-Rover Collective & Orchestration Layer
# ==============================================================================
add_library(astromesh_orchestration STATIC
    src/orchestration/rover_state_machine.cpp
    src/orchestration/multi_rover_collective.cpp
)

target_include_directories(astromesh_orchestration PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_orchestration PUBLIC
    astromesh_comms
    astromesh_optimization
    astromesh_verification
    astromesh_graph
    astromesh_geometry
    astromesh_core
)

# ==============================================================================
# Phase 7.14: Active Multi-Robot Coordination & Loop-Closure Rendezvous
# ==============================================================================
add_library(astromesh_coordination STATIC
    src/coordination/loop_closure_opportunity.cpp
    src/coordination/information_gain.cpp
    src/coordination/active_loop_scheduler.cpp
    src/coordination/rendezvous_protocol.cpp
)

target_include_directories(astromesh_coordination PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_coordination PUBLIC
    astromesh_verification
    astromesh_geometry
    astromesh_graph
    astromesh_core
)

# ==============================================================================
# Phase 7.15: Dynamic Map Compaction & Memory-Bounded Distributed SLAM
# ==============================================================================
add_library(astromesh_memory STATIC
    src/memory/memory_tracker.cpp
    src/memory/memory_budget_controller.cpp
    src/memory/map_compactor.cpp
    src/memory/landmark_retention.cpp
    src/memory/distributed_compaction_protocol.cpp
)

target_include_directories(astromesh_memory PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_memory PUBLIC
    astromesh_coordination
    astromesh_net
    astromesh_verification
    astromesh_geometry
    astromesh_graph
    astromesh_core
)

if(WIN32)
    target_link_libraries(astromesh_memory PUBLIC psapi)
endif()

# ==============================================================================
# Phase 7.16: Autonomous Exploration & Connectivity-Aware Path Planning
# ==============================================================================
add_library(astromesh_exploration STATIC
    src/exploration/frontier.cpp
    src/exploration/exploration_utility.cpp
    src/exploration/connectivity_path_planner.cpp
    src/exploration/multi_rover_coordinator.cpp
    src/exploration/integrated_exploration_agent.cpp
)

target_include_directories(astromesh_exploration PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_exploration PUBLIC
    astromesh_memory
    astromesh_coordination
    astromesh_net
    astromesh_verification
    astromesh_geometry
    astromesh_graph
    astromesh_core
)

# ==============================================================================
# Phase 5: Simulation Harness & Comparative Baselines
# ==============================================================================
add_library(astromesh_simulation STATIC
    src/simulation/synthetic_world.cpp
    src/simulation/swarm_trajectories.cpp
    src/simulation/observation_model.cpp
    src/simulation/voronoi_assignment.cpp
    src/simulation/baseline_astromesh_v1.cpp
    src/simulation/pipeline_astromesh_x.cpp
    src/simulation/failure_injector.cpp
)

target_include_directories(astromesh_simulation PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_simulation PUBLIC
    astromesh_comms
    astromesh_optimization
    astromesh_geometry
    astromesh_verification
    astromesh_graph
    astromesh_core
)



# ==============================================================================
# Testing Targets
# ==============================================================================
if(BUILD_TESTING)
    # Milestone 1.1: Landmark Key Tests
    add_executable(test_landmark_key
        tests/test_landmark_key.cpp
    )
    target_link_libraries(test_landmark_key PRIVATE
        astromesh_core
    )
    add_test(NAME test_landmark_key COMMAND test_landmark_key)

    # Milestone 1.2: Counting Bloom Filter Baseline Tests
    add_executable(test_counting_bloom_filter
        tests/test_counting_bloom_filter.cpp
    )
    target_link_libraries(test_counting_bloom_filter PRIVATE
        astromesh_core
    )
    add_test(NAME test_counting_bloom_filter COMMAND test_counting_bloom_filter)

    # Milestone 1.3: Invertible Bloom Lookup Table (IBLT) Tests
    add_executable(test_iblt
        tests/test_iblt.cpp
    )
    target_link_libraries(test_iblt PRIVATE
        astromesh_core
    )
    add_test(NAME test_iblt COMMAND test_iblt)

    # Milestone 1.4: Comparative Benchmark Suite & Property Tests
    add_executable(test_cbf_iblt_properties
        tests/test_cbf_iblt_properties.cpp
    )
    target_link_libraries(test_cbf_iblt_properties PRIVATE
        astromesh_core
    )
    add_test(NAME test_cbf_iblt_properties COMMAND test_cbf_iblt_properties)

    add_executable(benchmark_cbf_iblt
        tests/benchmark_cbf_iblt.cpp
    )
    target_link_libraries(benchmark_cbf_iblt PRIVATE
        astromesh_core
    )

    # Milestone 2.1: Topological Pose-Graph Data Structures Tests
    add_executable(test_pose_graph
        tests/test_pose_graph.cpp
    )
    target_link_libraries(test_pose_graph PRIVATE
        astromesh_graph
    )
    add_test(NAME test_pose_graph COMMAND test_pose_graph)

    # Milestone 2.2: Geometric Verification & Loop Closure Gating Tests
    add_executable(test_geometric_verifier
        tests/test_geometric_verifier.cpp
    )
    target_link_libraries(test_geometric_verifier PRIVATE
        astromesh_verification
    )
    add_test(NAME test_geometric_verifier COMMAND test_geometric_verifier)

    # Milestone 2.3: Disjoint Set Union & Partition Management Tests
    add_executable(test_disjoint_set
        tests/test_disjoint_set.cpp
    )
    target_link_libraries(test_disjoint_set PRIVATE
        astromesh_verification
    )
    add_test(NAME test_disjoint_set COMMAND test_disjoint_set)

    # Milestone 2.4: Perceptual Aliasing & Invalid Loop-Closure Rejection Tests
    add_executable(test_loop_closure_rejection
        tests/test_loop_closure_rejection.cpp
    )
    target_link_libraries(test_loop_closure_rejection PRIVATE
        astromesh_verification
    )
    add_test(NAME test_loop_closure_rejection COMMAND test_loop_closure_rejection)

    # Milestone 3.2: SE(2) Lie-Group Operations Tests
    add_executable(test_se2_lie_math
        tests/test_se2_lie_math.cpp
    )
    target_link_libraries(test_se2_lie_math PRIVATE
        astromesh_geometry
    )
    add_test(NAME test_se2_lie_math COMMAND test_se2_lie_math)

    # Milestone 3.2: Pose-Graph Objective Evaluation & Gradient Reference Tests
    add_executable(test_pose_graph_objective
        tests/test_pose_graph_objective.cpp
    )
    target_link_libraries(test_pose_graph_objective PRIVATE
        astromesh_optimization
        astromesh_geometry
    )
    add_test(NAME test_pose_graph_objective COMMAND test_pose_graph_objective)

    # Milestone 3.3: Centralized Reference Optimizer & Trajectory Metric Tests
    add_executable(test_centralized_optimizer
        tests/test_centralized_optimizer.cpp
    )
    target_include_directories(test_centralized_optimizer PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests"
    )
    target_link_libraries(test_centralized_optimizer PRIVATE
        astromesh_optimization
        astromesh_geometry
    )
    add_test(NAME test_centralized_optimizer COMMAND test_centralized_optimizer)

    # Milestone 3.3: Centralized Reference Optimizer Benchmark Executable
    add_executable(benchmark_centralized_optimizer
        tests/benchmark_centralized_optimizer.cpp
    )
    target_include_directories(benchmark_centralized_optimizer PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests"
    )
    target_link_libraries(benchmark_centralized_optimizer PRIVATE
        astromesh_optimization
        astromesh_geometry
    )

    # Milestone 3.4: CORD Dynamics & SE(2) Geometric Lie Integrator Tests
    add_executable(test_cord_integrator
        tests/test_cord_integrator.cpp
    )
    target_include_directories(test_cord_integrator PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests"
    )
    target_link_libraries(test_cord_integrator PRIVATE
        astromesh_optimization
        astromesh_geometry
    )
    add_test(NAME test_cord_integrator COMMAND test_cord_integrator)

    # Milestone 3.5: Distributed CORD Execution & Boundary State Prediction Tests
    add_executable(test_distributed_cord
        tests/test_distributed_cord.cpp
    )
    target_include_directories(test_distributed_cord PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests"
    )
    target_link_libraries(test_distributed_cord PRIVATE
        astromesh_optimization
        astromesh_geometry
    )
    add_test(NAME test_distributed_cord COMMAND test_distributed_cord)

    # Milestone 3.6: Distributed CORD Benchmarks, Ablations & Communication Profiling
    add_executable(benchmark_distributed_cord
        tests/benchmark_distributed_cord.cpp
    )
    target_include_directories(benchmark_distributed_cord PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests"
    )
    target_link_libraries(benchmark_distributed_cord PRIVATE
        astromesh_optimization
        astromesh_geometry
    )

    # Phase 4.1: Delay-Tolerant Boundary Filter & Stability Benchmark
    add_executable(benchmark_phase4_stability
        tests/benchmark_phase4_stability.cpp
    )
    target_include_directories(benchmark_phase4_stability PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests"
    )
    target_link_libraries(benchmark_phase4_stability PRIVATE
        astromesh_optimization
        astromesh_geometry
    )

    # Phase 4.2: DTN Store-and-Forward Buffer Tests
    add_executable(test_dtn_buffer
        tests/test_dtn_buffer.cpp
    )
    target_link_libraries(test_dtn_buffer PRIVATE
        astromesh_comms
    )
    add_test(NAME test_dtn_buffer COMMAND test_dtn_buffer)

    # Phase 4.2: Packet Budget & Quota Accounting Tests
    add_executable(test_packet_budget
        tests/test_packet_budget.cpp
    )
    target_link_libraries(test_packet_budget PRIVATE
        astromesh_comms
    )
    add_test(NAME test_packet_budget COMMAND test_packet_budget)

    # Phase 4.2: DTN Store-and-Forward Buffer & Byte Quota Benchmark
    add_executable(benchmark_dtn_buffer
        tests/benchmark_dtn_buffer.cpp
    )
    target_link_libraries(benchmark_dtn_buffer PRIVATE
        astromesh_comms
    )

    # Phase 4.3: Safe Keyframe Pruner & Bounded-Memory Tests
    add_executable(test_keyframe_pruner
        tests/test_keyframe_pruner.cpp
    )
    target_link_libraries(test_keyframe_pruner PRIVATE
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_keyframe_pruner COMMAND test_keyframe_pruner)

    # Phase 4.3: Safe Keyframe Pruning Benchmark
    add_executable(benchmark_keyframe_pruner
        tests/benchmark_keyframe_pruner.cpp
    )
    target_link_libraries(benchmark_keyframe_pruner PRIVATE
        astromesh_graph
        astromesh_core
    )

    # Phase 4.4: Dynamic Multi-Rover Collective & Orchestration Tests
    add_executable(test_rover_orchestration
        tests/test_rover_orchestration.cpp
    )
    target_link_libraries(test_rover_orchestration PRIVATE
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )
    add_test(NAME test_rover_orchestration COMMAND test_rover_orchestration)

    # Phase 4.4: Multi-Rover Collective Benchmark
    add_executable(benchmark_multi_rover_collective
        tests/benchmark_multi_rover_collective.cpp
    )
    target_link_libraries(benchmark_multi_rover_collective PRIVATE
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )

    # Phase 4.6: Reliable ACK & Multi-Hop DTN Benchmark
    add_executable(benchmark_phase46
        tests/benchmark_phase46.cpp
    )
    target_link_libraries(benchmark_phase46 PRIVATE
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )

    # Phase 5.1: Deterministic Synthetic World & Multi-Rover Trajectory Harness Tests
    add_executable(test_synthetic_world
        tests/test_synthetic_world.cpp
    )
    target_link_libraries(test_synthetic_world PRIVATE
        astromesh_simulation
        astromesh_geometry
        astromesh_verification
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_synthetic_world COMMAND test_synthetic_world)

    # Phase 5.2: AstroMesh v1 Baseline Pipeline Tests
    add_executable(test_baseline_astromesh_v1
        tests/test_baseline_astromesh_v1.cpp
    )
    target_link_libraries(test_baseline_astromesh_v1 PRIVATE
        astromesh_simulation
        astromesh_geometry
        astromesh_verification
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_baseline_astromesh_v1 COMMAND test_baseline_astromesh_v1)

    # Phase 5.3: AstroMesh-X Pipeline Integration Tests
    add_executable(test_pipeline_astromesh_x
        tests/test_pipeline_astromesh_x.cpp
    )
    target_link_libraries(test_pipeline_astromesh_x PRIVATE
        astromesh_simulation
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )
    add_test(NAME test_pipeline_astromesh_x COMMAND test_pipeline_astromesh_x)

    # Phase 5.4: Paired Monte Carlo Benchmark (Baseline vs AstroMesh-X)
    add_executable(benchmark_phase54
        tests/benchmark_phase54.cpp
    )
    target_link_libraries(benchmark_phase54 PRIVATE
        astromesh_simulation
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )

    # Phase 6.1: Controlled Communication Payload Ablation Study
    add_executable(benchmark_phase61
        tests/benchmark_phase61_ablations.cpp
    )
    target_link_libraries(benchmark_phase61 PRIVATE
        astromesh_simulation
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )

    # Phase 6.2: Packet Loss and Communication Delay Robustness Study
    add_executable(test_robustness_channel
        tests/test_robustness_channel.cpp
    )
    target_link_libraries(test_robustness_channel PRIVATE
        astromesh_simulation
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )
    add_test(NAME test_robustness_channel COMMAND test_robustness_channel)

    add_executable(benchmark_phase62
        tests/benchmark_phase62_robustness.cpp
    )
    target_link_libraries(benchmark_phase62 PRIVATE
        astromesh_simulation
        astromesh_orchestration
        astromesh_comms
        astromesh_optimization
        astromesh_verification
        astromesh_graph
        astromesh_geometry
        astromesh_core
    )
endif()

# ==============================================================================
# AstroMesh-X Standalone Simulation Engine CLI
# ==============================================================================
add_executable(sim_runner
    src/simulation/sim_runner.cpp
)
target_link_libraries(sim_runner PRIVATE
    astromesh_simulation
    astromesh_orchestration
    astromesh_comms
    astromesh_optimization
    astromesh_verification
    astromesh_graph
    astromesh_geometry
    astromesh_core
)

# ==============================================================================
# Independent Rover Networking & Distributed Reconciliation (Milestones 1, 2, 3)
# ==============================================================================
add_library(astromesh_net STATIC
    src/net/udp_socket.cpp
    src/net/message_frame.cpp
    src/net/peer_tracker.cpp
    src/net/channel_impairment.cpp
    src/net/rover_node.cpp
    src/net/distributed_reconciliation.cpp
    src/net/routing_table.cpp
    src/net/multihop_envelope.cpp
    src/net/multihop_router.cpp
    src/net/partition_reconciler.cpp
    src/net/adaptive_comms.cpp
    src/net/gauge_coordinator.cpp
    src/verification/global_consistency_auditor.cpp
)

target_include_directories(astromesh_net PUBLIC
    "${ASTROMESH_INCLUDE_DIR}"
)

target_link_libraries(astromesh_net PUBLIC
    astromesh_comms
    astromesh_core
    astromesh_graph
    astromesh_verification
    astromesh_optimization
    astromesh_geometry
)

if(WIN32)
    target_link_libraries(astromesh_net PUBLIC ws2_32)
endif()

add_executable(rover_daemon
    src/net/rover_daemon.cpp
)
target_link_libraries(rover_daemon PRIVATE
    astromesh_net
    astromesh_core
)

if(BUILD_TESTING)
    add_executable(test_rover_net
        tests/test_rover_net.cpp
    )
    target_link_libraries(test_rover_net PRIVATE
        astromesh_net
        astromesh_core
    )
    add_test(NAME test_rover_net COMMAND test_rover_net)

    add_executable(test_dtn_udp
        tests/test_dtn_udp.cpp
    )
    target_link_libraries(test_dtn_udp PRIVATE
        astromesh_net
        astromesh_comms
        astromesh_core
    )
    add_test(NAME test_dtn_udp COMMAND test_dtn_udp)

    add_executable(test_distributed_reconciliation
        tests/test_distributed_reconciliation.cpp
    )
    target_link_libraries(test_distributed_reconciliation PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
    )
    add_test(NAME test_distributed_reconciliation COMMAND test_distributed_reconciliation)

    add_executable(benchmark_distributed_system
        tests/benchmark_distributed_system.cpp
    )
    target_link_libraries(benchmark_distributed_system PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
    )

    add_executable(benchmark_distributed_noisy
        tests/benchmark_distributed_noisy.cpp
    )
    target_link_libraries(benchmark_distributed_noisy PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
        astromesh_comms
    )

    add_executable(test_audit_m77
        tests/test_audit_m77.cpp
    )
    target_link_libraries(test_audit_m77 PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
        astromesh_comms
    )
    add_test(NAME test_audit_m77 COMMAND test_audit_m77)

    # Milestone 7.10: Multi-Keyframe Trajectory SLAM, Distributed CORD, & Loop-Closure Precision
    add_executable(test_distributed_trajectory_slam
        tests/test_distributed_trajectory_slam.cpp
    )
    target_link_libraries(test_distributed_trajectory_slam PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
        astromesh_comms
    )
    add_test(NAME test_distributed_trajectory_slam COMMAND test_distributed_trajectory_slam)

    add_executable(benchmark_distributed_trajectory
        tests/benchmark_distributed_trajectory.cpp
    )
    target_link_libraries(benchmark_distributed_trajectory PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
        astromesh_comms
    )

    # Milestone 7.11: Multi-Hop Distributed SLAM Routing & Live UDP Dashboard Streaming
    add_executable(test_routing_table
        tests/test_routing_table.cpp
    )
    target_link_libraries(test_routing_table PRIVATE
        astromesh_net
        astromesh_core
    )
    add_test(NAME test_routing_table COMMAND test_routing_table)

    add_executable(test_multihop_envelope
        tests/test_multihop_envelope.cpp
    )
    target_link_libraries(test_multihop_envelope PRIVATE
        astromesh_net
        astromesh_core
    )
    add_test(NAME test_multihop_envelope COMMAND test_multihop_envelope)

    add_executable(test_multihop_router
        tests/test_multihop_router.cpp
    )
    target_link_libraries(test_multihop_router PRIVATE
        astromesh_net
        astromesh_core
    )
    add_test(NAME test_multihop_router COMMAND test_multihop_router)

    add_executable(test_multihop_slam_integration
        tests/test_multihop_slam_integration.cpp
    )
    target_link_libraries(test_multihop_slam_integration PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
        astromesh_comms
    )
    add_test(NAME test_multihop_slam_integration COMMAND test_multihop_slam_integration)

    add_executable(benchmark_multihop_mesh
        tests/benchmark_multihop_mesh.cpp
    )
    target_link_libraries(benchmark_multihop_mesh PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
        astromesh_comms
    )

    add_executable(benchmark_multihop_mesh_reconciled
        tests/benchmark_multihop_mesh_reconciled.cpp
    )
    target_link_libraries(benchmark_multihop_mesh_reconciled PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_verification
        astromesh_optimization
        astromesh_geometry
        astromesh_comms
    )
    add_executable(test_version_vector
        tests/test_version_vector.cpp
    )
    target_link_libraries(test_version_vector PRIVATE
        astromesh_graph
        astromesh_core
        astromesh_geometry
    )
    add_test(NAME test_version_vector COMMAND test_version_vector)

    add_executable(test_partition_reconciler
        tests/test_partition_reconciler.cpp
    )
    target_link_libraries(test_partition_reconciler PRIVATE
        astromesh_net
        astromesh_core
        astromesh_graph
        astromesh_geometry
    )
    add_test(NAME test_partition_reconciler COMMAND test_partition_reconciler)

    add_executable(test_conflict_graph_merger
        tests/test_conflict_graph_merger.cpp
    )
    target_link_libraries(test_conflict_graph_merger PRIVATE
        astromesh_graph
        astromesh_core
        astromesh_geometry
    )
    add_test(NAME test_conflict_graph_merger COMMAND test_conflict_graph_merger)

    add_executable(test_adaptive_comms
        tests/test_adaptive_comms.cpp
    )
    target_link_libraries(test_adaptive_comms PRIVATE
        astromesh_net
        astromesh_core
    )
    add_test(NAME test_adaptive_comms COMMAND test_adaptive_comms)

    add_executable(test_failure_injector
        tests/test_failure_injector.cpp
    )
    target_link_libraries(test_failure_injector PRIVATE
        astromesh_simulation
        astromesh_core
    )
    add_test(NAME test_failure_injector COMMAND test_failure_injector)

    add_executable(test_m712_partition_recovery
        tests/test_m712_partition_recovery.cpp
    )
    target_link_libraries(test_m712_partition_recovery PRIVATE
        astromesh_net
        astromesh_simulation
        astromesh_graph
        astromesh_core
        astromesh_geometry
    )
    add_test(NAME test_m712_partition_recovery COMMAND test_m712_partition_recovery)

    add_executable(benchmark_m712_partition_recovery
        tests/benchmark_m712_partition_recovery.cpp
    )
    target_link_libraries(benchmark_m712_partition_recovery PRIVATE
        astromesh_net
        astromesh_simulation
        astromesh_graph
        astromesh_core
        astromesh_geometry
    )

    add_executable(test_subgraph_alignment
        tests/test_subgraph_alignment.cpp
    )
    target_link_libraries(test_subgraph_alignment PRIVATE
        astromesh_geometry
        astromesh_verification
        astromesh_core
    )
    add_test(NAME test_subgraph_alignment COMMAND test_subgraph_alignment)

    add_executable(test_alignment_graph
        tests/test_alignment_graph.cpp
    )
    target_link_libraries(test_alignment_graph PRIVATE
        astromesh_geometry
        astromesh_verification
        astromesh_core
    )
    add_test(NAME test_alignment_graph COMMAND test_alignment_graph)

    add_executable(test_distributed_gauge
        tests/test_distributed_gauge.cpp
    )
    target_link_libraries(test_distributed_gauge PRIVATE
        astromesh_net
        astromesh_geometry
        astromesh_graph
        astromesh_verification
        astromesh_core
    )
    add_test(NAME test_distributed_gauge COMMAND test_distributed_gauge)

    add_executable(test_m713_partition_alignment
        tests/test_m713_partition_alignment.cpp
    )
    target_link_libraries(test_m713_partition_alignment PRIVATE
        astromesh_net
        astromesh_geometry
        astromesh_graph
        astromesh_verification
        astromesh_core
    )
    add_test(NAME test_m713_partition_alignment COMMAND test_m713_partition_alignment)

    add_executable(benchmark_m713_gauge_alignment
        tests/benchmark_m713_gauge_alignment.cpp
    )
    target_link_libraries(benchmark_m713_gauge_alignment PRIVATE
        astromesh_net
        astromesh_geometry
        astromesh_graph
        astromesh_verification
        astromesh_core
    )

    # Milestone 7.14 Test Targets
    add_executable(test_loop_closure_opportunity
        tests/test_loop_closure_opportunity.cpp
    )
    target_link_libraries(test_loop_closure_opportunity PRIVATE
        astromesh_coordination
        astromesh_verification
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_loop_closure_opportunity COMMAND test_loop_closure_opportunity)

    add_executable(test_information_gain
        tests/test_information_gain.cpp
    )
    target_link_libraries(test_information_gain PRIVATE
        astromesh_coordination
        astromesh_verification
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_information_gain COMMAND test_information_gain)

    add_executable(test_active_loop_scheduler
        tests/test_active_loop_scheduler.cpp
    )
    target_link_libraries(test_active_loop_scheduler PRIVATE
        astromesh_coordination
        astromesh_verification
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_active_loop_scheduler COMMAND test_active_loop_scheduler)

    add_executable(test_rendezvous_protocol
        tests/test_rendezvous_protocol.cpp
    )
    target_link_libraries(test_rendezvous_protocol PRIVATE
        astromesh_coordination
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_rendezvous_protocol COMMAND test_rendezvous_protocol)

    add_executable(test_m714_active_loop_integration
        tests/test_m714_active_loop_integration.cpp
    )
    target_link_libraries(test_m714_active_loop_integration PRIVATE
        astromesh_coordination
        astromesh_net
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_m714_active_loop_integration COMMAND test_m714_active_loop_integration)

    add_executable(benchmark_m714_active_loop_closure
        tests/benchmark_m714_active_loop_closure.cpp
    )
    target_link_libraries(benchmark_m714_active_loop_closure PRIVATE
        astromesh_coordination
        astromesh_net
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )

    # ==========================================================================
    # Phase 7.15 Tests & Benchmarks
    # ==========================================================================
    add_executable(test_memory_accounting
        tests/test_memory_accounting.cpp
    )
    target_link_libraries(test_memory_accounting PRIVATE
        astromesh_memory
        astromesh_net
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_memory_accounting COMMAND test_memory_accounting)

    add_executable(test_memory_budget_controller
        tests/test_memory_budget_controller.cpp
    )
    target_link_libraries(test_memory_budget_controller PRIVATE
        astromesh_memory
        astromesh_core
    )
    add_test(NAME test_memory_budget_controller COMMAND test_memory_budget_controller)

    add_executable(test_dynamic_map_compaction
        tests/test_dynamic_map_compaction.cpp
    )
    target_link_libraries(test_dynamic_map_compaction PRIVATE
        astromesh_memory
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_dynamic_map_compaction COMMAND test_dynamic_map_compaction)

    add_executable(test_landmark_retention
        tests/test_landmark_retention.cpp
    )
    target_link_libraries(test_landmark_retention PRIVATE
        astromesh_memory
        astromesh_verification
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_landmark_retention COMMAND test_landmark_retention)

    add_executable(test_distributed_compaction
        tests/test_distributed_compaction.cpp
    )
    target_link_libraries(test_distributed_compaction PRIVATE
        astromesh_memory
        astromesh_net
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_distributed_compaction COMMAND test_distributed_compaction)

    add_executable(test_compaction_partition_recovery
        tests/test_compaction_partition_recovery.cpp
    )
    target_link_libraries(test_compaction_partition_recovery PRIVATE
        astromesh_memory
        astromesh_net
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_compaction_partition_recovery COMMAND test_compaction_partition_recovery)

    add_executable(test_m715_memory_bounded_integration
        tests/test_m715_memory_bounded_integration.cpp
    )
    target_link_libraries(test_m715_memory_bounded_integration PRIVATE
        astromesh_memory
        astromesh_coordination
        astromesh_net
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_m715_memory_bounded_integration COMMAND test_m715_memory_bounded_integration)

    add_executable(benchmark_m715_memory_bounded_slam
        tests/benchmark_m715_memory_bounded_slam.cpp
    )
    target_link_libraries(benchmark_m715_memory_bounded_slam PRIVATE
        astromesh_memory
        astromesh_coordination
        astromesh_net
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )

    # ==========================================================================
    # Phase 7.16 Tests & Benchmarks: Autonomous Exploration & Connectivity Planning
    # ==========================================================================
    add_executable(test_frontier_detection
        tests/test_frontier_detection.cpp
    )
    target_link_libraries(test_frontier_detection PRIVATE
        astromesh_exploration
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_frontier_detection COMMAND test_frontier_detection)

    add_executable(test_exploration_utility
        tests/test_exploration_utility.cpp
    )
    target_link_libraries(test_exploration_utility PRIVATE
        astromesh_exploration
        astromesh_memory
        astromesh_core
    )
    add_test(NAME test_exploration_utility COMMAND test_exploration_utility)

    add_executable(test_connectivity_path_planner
        tests/test_connectivity_path_planner.cpp
    )
    target_link_libraries(test_connectivity_path_planner PRIVATE
        astromesh_exploration
        astromesh_memory
        astromesh_core
    )
    add_test(NAME test_connectivity_path_planner COMMAND test_connectivity_path_planner)

    add_executable(test_frontier_assignment
        tests/test_frontier_assignment.cpp
    )
    target_link_libraries(test_frontier_assignment PRIVATE
        astromesh_exploration
        astromesh_core
    )
    add_test(NAME test_frontier_assignment COMMAND test_frontier_assignment)

    add_executable(test_partition_autonomous_exploration
        tests/test_partition_autonomous_exploration.cpp
    )
    target_link_libraries(test_partition_autonomous_exploration PRIVATE
        astromesh_exploration
        astromesh_memory
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_partition_autonomous_exploration COMMAND test_partition_autonomous_exploration)

    add_executable(test_m716_exploration_integration
        tests/test_m716_exploration_integration.cpp
    )
    target_link_libraries(test_m716_exploration_integration PRIVATE
        astromesh_exploration
        astromesh_memory
        astromesh_coordination
        astromesh_net
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_m716_exploration_integration COMMAND test_m716_exploration_integration)

    add_executable(benchmark_m716_autonomous_exploration
        tests/benchmark_m716_autonomous_exploration.cpp
    )
    target_link_libraries(benchmark_m716_autonomous_exploration PRIVATE
        astromesh_exploration
        astromesh_memory
        astromesh_coordination
        astromesh_net
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )

    # --------------------------------------------------------------------------
    # Milestone 7.17: Scientific Benchmark Validation & Reproducibility Tests
    # --------------------------------------------------------------------------
    add_executable(test_m717_ground_truth_integrity
        tests/test_m717_ground_truth_integrity.cpp
    )
    target_link_libraries(test_m717_ground_truth_integrity PRIVATE
        astromesh_net
        astromesh_geometry
        astromesh_graph
        astromesh_core
        astromesh_optimization
    )
    add_test(NAME test_m717_ground_truth_integrity COMMAND test_m717_ground_truth_integrity)

    add_executable(test_m717_ate_alignment
        tests/test_m717_ate_alignment.cpp
    )
    target_link_libraries(test_m717_ate_alignment PRIVATE
        astromesh_optimization
        astromesh_geometry
        astromesh_graph
        astromesh_core
        astromesh_verification
    )
    add_test(NAME test_m717_ate_alignment COMMAND test_m717_ate_alignment)

    add_executable(test_m717_benchmark_fairness
        tests/test_m717_benchmark_fairness.cpp
    )
    target_link_libraries(test_m717_benchmark_fairness PRIVATE
        astromesh_simulation
        astromesh_geometry
        astromesh_core
    )
    add_test(NAME test_m717_benchmark_fairness COMMAND test_m717_benchmark_fairness)

    add_executable(test_m717_loop_closure_statistics
        tests/test_m717_loop_closure_statistics.cpp
    )
    target_link_libraries(test_m717_loop_closure_statistics PRIVATE
        astromesh_verification
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_m717_loop_closure_statistics COMMAND test_m717_loop_closure_statistics)

    add_executable(test_m717_network_accounting
        tests/test_m717_network_accounting.cpp
    )
    target_link_libraries(test_m717_network_accounting PRIVATE
        astromesh_net
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_m717_network_accounting COMMAND test_m717_network_accounting)

    add_executable(test_m717_memory_accounting
        tests/test_m717_memory_accounting.cpp
    )
    target_link_libraries(test_m717_memory_accounting PRIVATE
        astromesh_memory
        astromesh_graph
        astromesh_core
        psapi
    )
    add_test(NAME test_m717_memory_accounting COMMAND test_m717_memory_accounting)

    add_executable(test_m717_reproducibility
        tests/test_m717_reproducibility.cpp
    )
    target_link_libraries(test_m717_reproducibility PRIVATE
        astromesh_simulation
        astromesh_optimization
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_m717_reproducibility COMMAND test_m717_reproducibility)

    add_executable(test_m718_simulator_telemetry
        tests/test_m718_simulator_telemetry.cpp
    )
    target_link_libraries(test_m718_simulator_telemetry PRIVATE
        astromesh_simulation
        astromesh_geometry
        astromesh_graph
        astromesh_core
    )
    add_test(NAME test_m718_simulator_telemetry COMMAND test_m718_simulator_telemetry)
endif()
