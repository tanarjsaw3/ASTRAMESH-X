#include "astromesh/optimization/simulated_mesh_network.hpp"

#include <algorithm>

namespace astromesh::optimization {

SimulatedMeshNetwork::SimulatedMeshNetwork(const SimulatedNetworkConfig& config) noexcept
    : config_(config), rng_(config.random_seed) {}

void SimulatedMeshNetwork::reset(uint64_t initial_time_ns) {
    current_time_ns_ = initial_time_ns;
    next_packet_id_ = 1;
    stats_.reset();
    disconnected_links_.clear();
    while (!in_flight_queue_.empty()) {
        in_flight_queue_.pop();
    }
    rng_.seed(config_.random_seed);
}

void SimulatedMeshNetwork::set_link_state(uint16_t rover_a, uint16_t rover_b, bool connected) noexcept {
    if (rover_a == rover_b) return;
    auto pair = std::make_pair(std::min(rover_a, rover_b), std::max(rover_a, rover_b));
    if (connected) {
        disconnected_links_.erase(pair);
    } else {
        disconnected_links_.insert(pair);
    }
}

bool SimulatedMeshNetwork::is_link_connected(uint16_t rover_a, uint16_t rover_b) const noexcept {
    if (rover_a == rover_b) return true;
    auto pair = std::make_pair(std::min(rover_a, rover_b), std::max(rover_a, rover_b));
    return disconnected_links_.find(pair) == disconnected_links_.end();
}

bool SimulatedMeshNetwork::send_packet(const BoundaryStatePacket& packet) {
    if (!packet.is_valid()) {
        return false;
    }

    // Explicitly test serialization to account for physical wire bytes
    std::vector<uint8_t> wire_data = serialize_boundary_packet(packet);
    const size_t raw_bytes = wire_data.size();
    const size_t total_transport_bytes = raw_bytes + config_.transport_header_bytes;

    stats_.packets_sent++;
    stats_.raw_payload_bytes_sent += raw_bytes;
    stats_.transport_bytes_sent += total_transport_bytes;

    // Check link connectivity
    if (!is_link_connected(packet.sender_rover_id, packet.target_rover_id)) {
        stats_.packets_dropped++;
        return false;
    }

    // Check stochastic packet loss
    if (config_.packet_loss_rate > 0.0) {
        if (dist_unit_(rng_) < config_.packet_loss_rate) {
            stats_.packets_dropped++;
            return false;
        }
    }

    // Compute delivery timestamp
    uint64_t delay_ns = config_.latency_ns;
    if (config_.jitter_ns > 0) {
        delay_ns += static_cast<uint64_t>(dist_unit_(rng_) * static_cast<double>(config_.jitter_ns));
    }
    const uint64_t delivery_time_ns = current_time_ns_ + delay_ns;

    QueuedMessage msg;
    msg.delivery_time_ns = delivery_time_ns;
    msg.packet_id = next_packet_id_++;
    msg.target_rover_id = packet.target_rover_id;
    msg.packet = packet;

    in_flight_queue_.push(msg);

    // Stochastic packet duplication
    if (config_.duplicate_rate > 0.0) {
        if (dist_unit_(rng_) < config_.duplicate_rate) {
            QueuedMessage dup = msg;
            dup.packet_id = next_packet_id_++;
            dup.delivery_time_ns = delivery_time_ns + 1; // Delivered 1 ns later
            in_flight_queue_.push(dup);
            stats_.packets_duplicated++;
        }
    }

    return true;
}

std::vector<BoundaryStatePacket> SimulatedMeshNetwork::receive_packets(uint16_t target_rover_id) {
    std::vector<BoundaryStatePacket> delivered;
    std::vector<QueuedMessage> remaining_not_for_target;

    while (!in_flight_queue_.empty()) {
        const auto& top = in_flight_queue_.top();
        if (top.delivery_time_ns > current_time_ns_) {
            // Not ready for delivery yet
            break;
        }

        QueuedMessage msg = top;
        in_flight_queue_.pop();

        if (msg.target_rover_id == target_rover_id) {
            // Check if link is still connected at delivery time
            if (is_link_connected(msg.packet.sender_rover_id, msg.target_rover_id)) {
                delivered.push_back(msg.packet);
                stats_.packets_delivered++;
                stats_.raw_payload_bytes_delivered += BOUNDARY_PACKET_WIRE_SIZE;
                stats_.transport_bytes_delivered += (BOUNDARY_PACKET_WIRE_SIZE + config_.transport_header_bytes);
            } else {
                stats_.packets_dropped++;
            }
        } else {
            // Packet is ready for another rover; hold and re-enqueue
            remaining_not_for_target.push_back(msg);
        }
    }

    // Put back packets belonging to other rovers
    for (auto& msg : remaining_not_for_target) {
        in_flight_queue_.push(std::move(msg));
    }

    return delivered;
}

} // namespace astromesh::optimization
