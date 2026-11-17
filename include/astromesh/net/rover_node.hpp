#pragma once

#include "astromesh/comms/dtn_buffer.hpp"
#include "astromesh/net/channel_impairment.hpp"
#include "astromesh/net/message_frame.hpp"
#include "astromesh/net/multihop_envelope.hpp"
#include "astromesh/net/multihop_router.hpp"
#include "astromesh/net/peer_tracker.hpp"
#include "astromesh/net/routing_table.hpp"
#include "astromesh/net/udp_socket.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace astromesh::net {

struct RoverNodeConfig {
    uint16_t rover_id{1};
    uint16_t listen_port{19001};
    std::string listen_ip{"127.0.0.1"};
    std::vector<Endpoint> seed_peers;
    uint32_t heartbeat_interval_ms{200};
    uint32_t peer_timeout_ms{1000};

    // DTN & Network Impairment options
    bool enable_dtn{true};
    uint32_t transport_framing_bytes{20}; ///< Overhead for transport/IP layer accounting
    comms::DtnQueueConfig dtn_config;
    ChannelImpairmentConfig impairment_config;

    // Multi-Hop Routing options (Milestone 7.11)
    bool enable_multihop{true};
    uint32_t routing_advertisement_interval_ms{300};
    uint8_t default_max_hops{5};
};

struct RoverNodeStats {
    uint64_t packets_sent{0};
    uint64_t packets_received{0};
    uint64_t bytes_sent{0};
    uint64_t bytes_received{0};
    uint64_t duplicates_dropped{0};
    uint64_t corrupted_dropped{0};
    uint64_t heartbeats_sent{0};
    uint64_t heartbeats_received{0};
};

/**
 * @brief Wire-level byte and delivery accounting metrics.
 */
struct RoverWireStats {
    uint64_t payload_bytes_sent{0};
    uint64_t header_bytes_sent{0};
    uint64_t framing_bytes_sent{0};
    uint64_t data_wire_bytes_sent{0};

    uint64_t ack_wire_bytes_sent{0};
    uint64_t ack_wire_bytes_received{0};
    uint64_t total_wire_bytes_sent{0};

    uint64_t dtn_messages_enqueued{0};
    uint64_t dtn_messages_delivered{0};
    uint64_t dtn_retransmissions{0};
    uint64_t acks_sent{0};
    uint64_t acks_received{0};
    uint64_t duplicate_deliveries_suppressed{0};

    void reset() noexcept {
        *this = RoverWireStats{};
    }
};

/**
 * @brief Standalone rover communication node executing an independent event loop over UDP
 *        with integrated DTN store-and-forward, ACKs, retransmissions, and wire byte accounting.
 */
class RoverNode {
public:
    explicit RoverNode(const RoverNodeConfig& config);
    ~RoverNode();

    // Lifecycle
    bool start();
    void stop();
    bool is_running() const noexcept { return m_running; }

    /**
     * @brief Polls incoming UDP packets, processes DTN retries, peer tracking, and heartbeats.
     * @param now_ns Current monotonic time in nanoseconds. If 0, queries std::chrono.
     * @return Number of valid non-heartbeat application messages processed.
     */
    size_t poll_events(uint64_t now_ns = 0);

    /**
     * @brief Transmits a raw message frame to all known active peers.
     */
    int64_t broadcast_message(MessageType type, const std::vector<uint8_t>& payload, uint64_t now_ns = 0);

    /**
     * @brief Transmits a raw message frame to a specific target rover.
     */
    int64_t send_message_to(uint16_t target_rover_id, MessageType type,
                            const std::vector<uint8_t>& payload, uint64_t now_ns = 0);

    /**
     * @brief Enqueues a DTN message for reliable store-and-forward transmission with ACK tracking.
     * @return Unique 64-bit message ID (> 0), or 0 if rejected by buffer capacity/quota.
     */
    uint64_t enqueue_dtn_message(
        uint16_t target_rover_id,
        MessageType type,
        const std::vector<uint8_t>& payload,
        comms::PriorityLevel priority = comms::PriorityLevel::Normal,
        uint64_t ttl_ns = 60'000'000'000ULL,
        uint64_t now_ns = 0);

    /**
     * @brief Registers callback for incoming application messages (non-heartbeat).
     */
    void set_message_handler(std::function<void(const MessageFrame&, const Endpoint&)> handler) {
        m_message_handler = std::move(handler);
    }

    uint16_t get_rover_id() const noexcept { return m_config.rover_id; }
    uint16_t get_listen_port() const noexcept { return m_socket.get_bound_port(); }
    const PeerTracker& get_peer_tracker() const noexcept { return m_peers; }
    PeerTracker& get_peer_tracker() noexcept { return m_peers; }

    RoverNodeStats get_stats() const noexcept { return m_stats; }
    RoverWireStats get_wire_stats() const noexcept { return m_wire_stats; }

    const comms::DtnBuffer& get_dtn_buffer() const noexcept { return m_dtn_buffer; }
    comms::DtnBuffer& get_dtn_buffer() noexcept { return m_dtn_buffer; }

    const ChannelImpairment& get_channel_impairment() const noexcept { return m_impairment; }
    ChannelImpairment& get_channel_impairment() noexcept { return m_impairment; }

    // Multi-Hop Routing Accessors (Milestone 7.11)
    const RoutingTable& get_routing_table() const noexcept { return m_routing_table; }
    RoutingTable& get_routing_table() noexcept { return m_routing_table; }

    const MultihopRouter& get_multihop_router() const noexcept { return m_multihop_router; }
    MultihopRouter& get_multihop_router() noexcept { return m_multihop_router; }

    /**
     * @brief Explicitly originates a multi-hop mesh message with end-to-end delivery tracking.
     */
    bool originate_multihop(uint16_t final_target_id, MessageType type,
                            const std::vector<uint8_t>& payload, uint64_t now_ns = 0);

private:
    uint64_t get_now_ns() const noexcept;
    void send_heartbeats(uint64_t now_ns);
    void send_routing_advertisements(uint64_t now_ns);
    void process_dtn_outgoing(uint64_t now_ns);
    void send_ack(uint16_t target_rover_id, uint64_t message_id, uint64_t now_ns);
    void send_custody_ack(uint16_t upstream_rover_id, uint64_t message_id, uint64_t now_ns);
    void send_e2e_ack(uint16_t origin_rover_id, uint64_t message_id, uint64_t now_ns);
    bool forward_multihop_frame(const MultihopEnvelope& env, uint16_t next_hop, uint64_t now_ns);
    void handle_multihop_local_delivery(const MultihopEnvelope& env, uint64_t now_ns);
    int64_t transmit_raw_frame(const MessageFrame& frame, const Endpoint& dest, uint64_t now_ns);

    RoverNodeConfig m_config;
    UdpSocket m_socket;
    PeerTracker m_peers;
    comms::DtnBuffer m_dtn_buffer;
    ChannelImpairment m_impairment;
    RoverNodeStats m_stats;
    RoverWireStats m_wire_stats;

    // Multi-Hop Routing State (Milestone 7.11)
    RoutingTable m_routing_table;
    MultihopRouter m_multihop_router;
    uint64_t m_last_routing_adv_ns{0};
    uint64_t m_last_route_prune_ns{0};
    std::unordered_map<uint64_t, uint64_t> m_delivered_message_ids;

    uint32_t m_next_sequence{1};
    uint64_t m_next_dtn_msg_counter{0};
    uint64_t m_last_heartbeat_ns{0};
    bool m_running{false};
    std::function<void(const MessageFrame&, const Endpoint&)> m_message_handler;
};

} // namespace astromesh::net
