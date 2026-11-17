#pragma once

#include "astromesh/comms/dtn_buffer.hpp"
#include "astromesh/comms/packet_budget.hpp"
#include "astromesh/core/counting_bloom_filter.hpp"
#include "astromesh/core/iblt.hpp"
#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/keyframe_pruner.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/graph/disjoint_set.hpp"
#include "astromesh/optimization/boundary_filter.hpp"
#include "astromesh/optimization/boundary_packet.hpp"
#include "astromesh/optimization/distributed_cord_node.hpp"
#include "astromesh/optimization/simulated_mesh_network.hpp"
#include "astromesh/orchestration/rover_state_machine.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace astromesh::orchestration {

/**
 * @brief Configuration parameters for a single simulated rover.
 */
struct SimulatedRoverConfig {
    uint16_t rover_id{1};
    geometry::SE2 initial_pose{0.0, 0.0, 0.0};
    RoverStateMachineConfig state_machine_config{};
    comms::DtnQueueConfig dtn_queue_config{};
    comms::ByteBudgetConfig budget_config{};
    graph::PrunerConfig pruner_config{};
    optimization::BoundaryFilterOptions filter_options{};
    size_t iblt_cell_count{32};
    size_t iblt_probes{3};
    size_t cbf_capacity{100};
};

/**
 * @brief Individual simulated rover encapsulating state machine, pose graph,
 *        DTN buffer, byte budget manager, and local SLAM data structures.
 */
class SimulatedRover {
public:
    explicit SimulatedRover(const SimulatedRoverConfig& config);

    uint16_t rover_id() const noexcept { return config_.rover_id; }
    RoverState current_state() const noexcept { return state_machine_.current_state(); }
    RoverStateMachine& state_machine() noexcept { return state_machine_; }
    const RoverStateMachine& state_machine() const noexcept { return state_machine_; }

    graph::PoseGraph& pose_graph() noexcept { return pose_graph_; }
    const graph::PoseGraph& pose_graph() const noexcept { return pose_graph_; }

    graph::SubgraphPartitionManager& partition_manager() noexcept { return partition_manager_; }
    const graph::SubgraphPartitionManager& partition_manager() const noexcept { return partition_manager_; }

    comms::DtnBuffer& dtn_buffer() noexcept { return dtn_buffer_; }
    const comms::DtnBuffer& dtn_buffer() const noexcept { return dtn_buffer_; }

    comms::PacketBudgetManager& budget_manager() noexcept { return budget_manager_; }
    const comms::PacketBudgetManager& budget_manager() const noexcept { return budget_manager_; }

    core::InvertibleBloomFilter& iblt() noexcept { return iblt_; }
    const core::InvertibleBloomFilter& iblt() const noexcept { return iblt_; }

    core::CountingBloomFilter& cbf() noexcept { return cbf_; }
    const core::CountingBloomFilter& cbf() const noexcept { return cbf_; }

    optimization::BoundaryStateFilter& boundary_filter() noexcept { return boundary_filter_; }
    const optimization::BoundaryStateFilter& boundary_filter() const noexcept { return boundary_filter_; }

    uint64_t last_peer_contact_ns() const noexcept { return last_peer_contact_ns_; }
    void update_last_peer_contact_ns(uint64_t timestamp_ns) noexcept {
        last_peer_contact_ns_ = timestamp_ns;
    }

    uint32_t current_keyframe_index() const noexcept { return current_keyframe_index_; }
    geometry::SE2 current_pose() const noexcept { return current_pose_; }

    /**
     * @brief Adds a new odometry keyframe node to the local pose graph,
     *        records observed landmarks into CBF and IBLT, and checks keyframe admission.
     */
    bool add_odometry_step(
        const geometry::SE2& delta_pose,
        uint64_t timestamp_ns,
        const std::vector<core::LandmarkKey>& landmarks = {});

    /**
     * @brief Adds an unverified inter-rover loop candidate edge to the local pose graph.
     */
    bool add_candidate_loop_closure(
        const graph::NodeId& source,
        const graph::NodeId& target,
        const geometry::SE2& relative_pose,
        const graph::InformationMatrix2D& info);

    /**
     * @brief Attempts safe keyframe pruning under configured memory bounds.
     */
    graph::PruningResult prune_keyframes_if_needed();

    void mark_quota_denied(uint64_t now_ns) noexcept {
        quota_denied_in_window_ = true;
        last_quota_denial_window_ns_ = budget_manager_.current_window_start_ns();
    }

    bool is_quota_constrained(uint64_t now_ns) noexcept {
        if (!quota_denied_in_window_) {
            return false;
        }
        const uint64_t win_dur = budget_manager_.config().window_duration_ns;
        if (win_dur > 0 && now_ns >= last_quota_denial_window_ns_ + win_dur) {
            quota_denied_in_window_ = false;
            return false;
        }
        return true;
    }

    // --- Counters & Telemetry ---
    size_t sync_attempts{0};
    size_t sync_successes{0};
    size_t sync_incomplete_decodes{0};
    size_t sync_budget_denials{0};
    size_t keyframe_prunes_applied{0};
    size_t keyframe_prunes_rejected{0};
    size_t messages_processed{0};

private:
    SimulatedRoverConfig config_;
    RoverStateMachine state_machine_;
    graph::PoseGraph pose_graph_;
    graph::SubgraphPartitionManager partition_manager_;
    graph::KeyframePruner keyframe_pruner_;
    comms::PacketBudgetManager budget_manager_;
    comms::DtnBuffer dtn_buffer_;
    core::InvertibleBloomFilter iblt_;
    core::CountingBloomFilter cbf_;
    optimization::BoundaryStateFilter boundary_filter_;

    uint32_t current_keyframe_index_{0};
    geometry::SE2 current_pose_{0.0, 0.0, 0.0};
    uint64_t last_peer_contact_ns_{0};
    uint64_t last_keyframe_time_ns_{0};
    geometry::SE2 last_retained_pose_{0.0, 0.0, 0.0};
    uint64_t last_quota_denial_window_ns_{0};
    bool quota_denied_in_window_{false};
};

/**
 * @brief Comprehensive deterministic metrics for a multi-rover collective simulation trial.
 */
struct CollectiveMetrics {
    std::string scenario_name;
    uint64_t seed{42};
    size_t rover_count{0};
    double duration_ms{0.0};
    double step_ms{0.0};
    double latency_ms{0.0};
    double packet_loss_rate{0.0};
    double partition_duration_ms{0.0};

    // Message counts
    size_t messages_generated{0};
    size_t messages_admitted{0};
    size_t messages_sent{0};
    size_t initial_transmission_attempts{0};
    size_t messages_retried{0};
    size_t messages_delivered{0};
    size_t unique_messages_delivered{0};
    size_t duplicate_deliveries{0};
    size_t messages_processed{0};
    size_t messages_expired{0};
    size_t messages_dropped{0};
    size_t messages_rejected{0};
    size_t retries_exhausted{0};
    size_t in_flight_messages{0};

    // Byte counts
    size_t payload_bytes_sent{0};
    size_t protocol_bytes_sent{0};
    size_t framing_bytes_sent{0};
    size_t total_wire_bytes_sent{0};
    size_t total_wire_bytes_delivered{0};

    // Phase 4.6 Reliable Delivery & Multi-Hop DTN Metrics
    size_t hop_data_attempts{0};
    size_t hop_data_delivered{0};
    size_t hop_data_losses{0};
    size_t ack_transmission_attempts{0};
    size_t ack_deliveries{0};
    size_t ack_losses{0};
    size_t retransmissions_missing_ack{0};
    size_t unique_messages_processed{0};
    size_t duplicate_processing_attempts{0};
    size_t hop_limit_exhausted{0};
    size_t intermediate_queued_messages{0};
    size_t ack_wire_bytes_sent{0};
    size_t ack_wire_bytes_delivered{0};
    double end_to_end_completion_ratio{0.0};

    // Telemetry & Filtering
    size_t boundary_packets_filtered{0};

    // State machine & Synchronization
    size_t state_transitions_total{0};
    size_t sync_attempts{0};
    size_t sync_successes{0};
    size_t sync_incomplete_decodes{0};
    size_t sync_budget_denials{0};

    // Queue & Resource metrics
    size_t queue_peak_messages{0};
    size_t queue_peak_payload_bytes{0};
    size_t byte_budget_denials{0};

    // Graph & Pruning
    size_t final_connected_components{0};
    size_t verified_inter_rover_closures{0};
    size_t keyframe_prunes_applied{0};
    size_t keyframe_prunes_rejected{0};
};

/**
 * @brief Configuration parameters for a multi-rover collective simulation.
 */
struct CollectiveSimulationConfig {
    std::string scenario_name{"DefaultCollective"};
    std::vector<SimulatedRoverConfig> rover_configs;
    optimization::SimulatedNetworkConfig network_config{};
    size_t sync_byte_limit{1500};
    bool enable_dtn_retries{true};
    bool enable_boundary_filtering{true};
    bool enable_keyframe_pruning{true};
    uint64_t random_seed{42};

    // Phase 4.6 Reliable Delivery & Routing Extensions (Opt-in to preserve exact Phase 4.4/4.5 baselines)
    bool enable_explicit_acks{false};
    bool enable_multihop_routing{false};
    uint8_t default_max_hops{5};
    double ack_packet_loss_rate{-1.0}; // -1.0 means inherit network_config.packet_loss_rate
    std::map<std::pair<uint16_t, uint16_t>, uint16_t> static_routes;
};

/**
 * @brief Multi-rover collective orchestrator integrating state machines, DTN buffers,
 *        discrete-event mesh communications, landmark reconciliation, and pose graphs.
 */
class MultiRoverCollective {
public:
    explicit MultiRoverCollective(const CollectiveSimulationConfig& config);

    uint64_t current_time_ns() const noexcept { return current_time_ns_; }
    const CollectiveSimulationConfig& config() const noexcept { return config_; }

    SimulatedRover* get_rover(uint16_t rover_id) noexcept;
    const SimulatedRover* get_rover(uint16_t rover_id) const noexcept;
    const std::map<uint16_t, std::unique_ptr<SimulatedRover>>& rovers() const noexcept { return rovers_; }

    /**
     * @brief Configures bilateral link state between two rovers in the network.
     */
    void set_link_state(uint16_t rover_a, uint16_t rover_b, bool connected) noexcept;
    bool is_link_connected(uint16_t rover_a, uint16_t rover_b) const noexcept;

    /**
     * @brief Configures a static next-hop route for (source, destination).
     */
    void set_static_route(uint16_t source, uint16_t destination, uint16_t next_hop) noexcept;

    /**
     * @brief Resolves the next-hop rover towards destination.
     *        Returns std::nullopt if no connected path is currently known.
     */
    std::optional<uint16_t> find_next_hop(uint16_t current_rover, uint16_t target_rover) const;

    /**
     * @brief Enqueues a message from sender to target via DTN buffer and budget manager.
     */
    comms::EnqueueResult enqueue_message(
        uint16_t sender_rover_id,
        uint16_t target_rover_id,
        comms::DtnMessageType message_type,
        comms::PriorityLevel priority,
        const std::vector<uint8_t>& payload,
        uint64_t ttl_ns,
        uint32_t sequence_number = 0,
        uint8_t max_hops = 5);

    /**
     * @brief Advances simulation time by dt_ns and processes discrete-event actions:
     *        DTN dispatch, mesh delivery, timeout evaluation, and state transitions.
     */
    void step(uint64_t dt_ns);

    /**
     * @brief Executes simulation run for duration_ns in discrete steps of step_dt_ns.
     */
    void run(uint64_t duration_ns, uint64_t step_dt_ns);

    /**
     * @brief Executes a landmark reconciliation and geometric verification procedure
     *        between two rovers.
     * @param rover_a Initiating rover.
     * @param rover_b Peer rover.
     * @param candidate_correspondences Optional landmark point correspondences for geometric verification.
     * @return true if synchronization successfully reconciled and promoted verified closure.
     */
    bool perform_synchronization(
        uint16_t rover_a,
        uint16_t rover_b,
        const std::vector<verification::PointCorrespondence2D>& candidate_correspondences = {});

    /**
     * @brief Collects summary metrics across all rovers and communication subsystems.
     */
    CollectiveMetrics collect_metrics() const;

private:
    struct InTransitDtnMessage {
        uint64_t delivery_time_ns{0};
        uint64_t transit_id{0};
        uint16_t hop_sender_id{0};
        uint16_t hop_target_id{0};
        comms::DtnMessage message;

        bool operator>(const InTransitDtnMessage& other) const noexcept {
            if (delivery_time_ns != other.delivery_time_ns) {
                return delivery_time_ns > other.delivery_time_ns;
            }
            return transit_id > other.transit_id;
        }
    };

    CollectiveSimulationConfig config_;
    uint64_t current_time_ns_{0};
    uint64_t next_message_seq_{1};
    uint64_t next_transit_id_{1};

    std::map<uint16_t, std::unique_ptr<SimulatedRover>> rovers_;
    optimization::SimulatedMeshNetwork network_;
    std::set<std::pair<uint16_t, uint16_t>> disconnected_links_;
    std::map<std::pair<uint16_t, uint16_t>, uint16_t> static_routes_;

    // Transit queue for discrete-event DTN messages
    std::priority_queue<InTransitDtnMessage, std::vector<InTransitDtnMessage>, std::greater<InTransitDtnMessage>> in_transit_messages_;

    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> dist_unit_{0.0, 1.0};

    // Collective telemetry counters
    size_t total_messages_generated_{0};
    size_t total_messages_admitted_{0};
    size_t total_messages_sent_{0};
    size_t total_initial_attempts_{0};
    size_t total_messages_retried_{0};
    size_t total_messages_delivered_{0};
    size_t total_unique_delivered_{0};
    size_t total_duplicate_deliveries_{0};
    size_t total_messages_expired_{0};
    size_t total_messages_dropped_{0};
    size_t total_messages_rejected_{0};
    size_t total_boundary_packets_filtered_{0};
    size_t total_payload_bytes_sent_{0};
    size_t total_protocol_bytes_sent_{0};
    size_t total_framing_bytes_sent_{0};
    size_t total_wire_bytes_sent_{0};
    size_t total_wire_bytes_delivered_{0};
    size_t peak_queue_messages_{0};
    size_t peak_queue_payload_bytes_{0};
    uint64_t last_step_dt_ns_{10'000'000};

    // Phase 4.6 Telemetry Counters
    size_t hop_data_attempts_{0};
    size_t hop_data_delivered_{0};
    size_t hop_data_losses_{0};
    size_t ack_transmission_attempts_{0};
    size_t ack_deliveries_{0};
    size_t ack_losses_{0};
    size_t retransmissions_missing_ack_{0};
    size_t unique_messages_processed_{0};
    size_t duplicate_processing_attempts_{0};
    size_t hop_limit_exhausted_{0};
    size_t ack_wire_bytes_sent_{0};
    size_t ack_wire_bytes_delivered_{0};

    void dispatch_outgoing_dtn_messages();
    void deliver_inbound_dtn_messages();
    void process_received_message(SimulatedRover& destination, const comms::DtnMessage& message);
};

} // namespace astromesh::orchestration
