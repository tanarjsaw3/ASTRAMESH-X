#include "astromesh/optimization/boundary_filter.hpp"

#include <algorithm>
#include <cmath>

namespace astromesh::optimization {

BoundaryStateFilter::BoundaryStateFilter(const BoundaryFilterOptions& options)
    : options_(options) {
}

bool BoundaryStateFilter::process_packet(const BoundaryStatePacket& packet, uint64_t reception_time_ns) {
    // 1. Basic packet structural and checksum validation
    if (!packet.is_valid()) {
        rejected_packets_++;
        return false;
    }

    // 2. Protocol version check
    if (packet.protocol_version != BOUNDARY_PROTOCOL_VERSION) {
        rejected_packets_++;
        return false;
    }

    // 3. Finite-value verification for physical states
    if (!std::isfinite(packet.x) || !std::isfinite(packet.y) || !std::isfinite(packet.theta) ||
        !std::isfinite(packet.vx) || !std::isfinite(packet.vy) || !std::isfinite(packet.omega)) {
        rejected_packets_++;
        return false;
    }

    const graph::NodeId nid = packet.node_id();
    auto& sample = samples_[nid];

    // 4. Sequence number ordering check (RFC 1982 modular comparison)
    if (sample.has_sample) {
        if (!is_newer_sequence(packet.sequence_number, sample.sequence_number)) {
            rejected_packets_++;
            return false;
        }
    }

    // 5. Velocity filtering, saturation, and deadband
    const geometry::Twist2D raw_vel = packet.velocity();
    geometry::Twist2D filt_vel = raw_vel;

    if (options_.enable_filtering) {
        // A. Exponential moving average (EMA) low-pass filtering
        if (sample.has_sample) {
            const double alpha_v = options_.velocity_smoothing_factor;
            filt_vel.vx = (1.0 - alpha_v) * sample.filtered_velocity.vx + alpha_v * raw_vel.vx;
            filt_vel.vy = (1.0 - alpha_v) * sample.filtered_velocity.vy + alpha_v * raw_vel.vy;
            filt_vel.omega = (1.0 - alpha_v) * sample.filtered_velocity.omega + alpha_v * raw_vel.omega;
        } else {
            filt_vel = raw_vel;
        }

        // B. Linear velocity norm clamping
        double v_norm = std::sqrt(filt_vel.vx * filt_vel.vx + filt_vel.vy * filt_vel.vy);
        if (v_norm > options_.max_linear_velocity_m_s && v_norm > 1e-12) {
            double scale = options_.max_linear_velocity_m_s / v_norm;
            filt_vel.vx *= scale;
            filt_vel.vy *= scale;
            v_norm = options_.max_linear_velocity_m_s;
        }

        // C. Angular velocity saturation
        filt_vel.omega = std::clamp(filt_vel.omega, -options_.max_angular_velocity_rad_s, options_.max_angular_velocity_rad_s);

        // D. Quiescent noise deadbands
        if (v_norm < options_.velocity_deadband_linear) {
            filt_vel.vx = 0.0;
            filt_vel.vy = 0.0;
        }
        if (std::abs(filt_vel.omega) < options_.velocity_deadband_angular) {
            filt_vel.omega = 0.0;
        }
    }

    // 6. Record sample state
    sample.node_id = nid;
    sample.sender_rover_id = packet.sender_rover_id;
    sample.target_rover_id = packet.target_rover_id;
    sample.latest_pose = packet.pose();
    sample.raw_velocity = raw_vel;
    sample.filtered_velocity = filt_vel;
    sample.packet_timestamp_ns = packet.timestamp_ns;
    sample.reception_timestamp_ns = reception_time_ns;
    sample.sequence_number = packet.sequence_number;
    sample.has_sample = true;

    accepted_packets_++;
    return true;
}

bool BoundaryStateFilter::get_estimated_state(
    const graph::NodeId& neighbor_id,
    uint64_t query_time_ns,
    geometry::SE2& out_pose,
    geometry::Twist2D& out_velocity,
    bool& out_is_fallback) const {

    auto it = samples_.find(neighbor_id);
    if (it == samples_.end() || !it->second.has_sample) {
        return false;
    }

    const FilteredBoundarySample& sample = it->second;

    const uint64_t age_ns = (query_time_ns >= sample.packet_timestamp_ns)
        ? (query_time_ns - sample.packet_timestamp_ns)
        : 0;
    const double age_s = static_cast<double>(age_ns) * 1e-9;

    // Check prediction horizon limit
    const bool exceeds_horizon = options_.enable_filtering && (age_s > options_.max_prediction_horizon_s);

    if (exceeds_horizon || age_s <= 0.0) {
        // Clean fallback to Zero-Order Hold (ZOH)
        out_pose = sample.latest_pose;
        out_velocity = geometry::Twist2D(0.0, 0.0, 0.0);
        out_is_fallback = true;
        if (exceeds_horizon) {
            fallback_count_++;
        }
        return true;
    }

    // SE(2) Exponential map extrapolation using filtered velocity
    const geometry::Twist2D delta_twist(
        age_s * sample.filtered_velocity.vx,
        age_s * sample.filtered_velocity.vy,
        age_s * sample.filtered_velocity.omega
    );

    out_pose = sample.latest_pose * geometry::SE2::exp(delta_twist);
    out_velocity = sample.filtered_velocity;
    out_is_fallback = false;
    return true;
}

const FilteredBoundarySample* BoundaryStateFilter::get_sample(const graph::NodeId& neighbor_id) const {
    auto it = samples_.find(neighbor_id);
    if (it == samples_.end() || !it->second.has_sample) {
        return nullptr;
    }
    return &it->second;
}

double BoundaryStateFilter::compute_damping_multiplier(double delay_s) const {
    if (!options_.enable_adaptive_damping) {
        return 1.0;
    }
    const double d = std::max(0.0, delay_s);
    const double mult = 1.0 + options_.adaptive_damping_alpha * d;
    return std::clamp(mult, 1.0, options_.max_damping_multiplier);
}

void BoundaryStateFilter::reset(const graph::NodeId& neighbor_id) {
    samples_.erase(neighbor_id);
}

void BoundaryStateFilter::clear() {
    samples_.clear();
    accepted_packets_ = 0;
    rejected_packets_ = 0;
    fallback_count_ = 0;
}

} // namespace astromesh::optimization
