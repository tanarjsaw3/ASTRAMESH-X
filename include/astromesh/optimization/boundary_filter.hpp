#pragma once

#include "astromesh/geometry/se2.hpp"
#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/optimization/boundary_packet.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace astromesh::optimization {

/**
 * @brief Configuration parameters for boundary-state filtering and delay mitigation.
 *
 * Every parameter has defined units, validation bounds, defaults, and documented effects.
 */
struct BoundaryFilterOptions {
    /**
     * @brief Master toggle for boundary-state filtering.
     * Default: false (preserves exact baseline backward-compatibility when disabled).
     */
    bool enable_filtering{false};

    /**
     * @brief First-order exponential moving average (EMA) smoothing factor for velocity.
     * Formula: xi_{filt} = (1 - alpha_v) * xi_{prev} + alpha_v * xi_{raw}
     * Units: Dimensionless in range (0.0, 1.0].
     * 1.0 = raw pass-through (no smoothing). Lower values increase low-pass damping.
     * Default: 0.50.
     */
    double velocity_smoothing_factor{0.50};

    /**
     * @brief Maximum allowable linear velocity norm sqrt(vx^2 + vy^2) in rover body frame.
     * Exceeding velocities are scaled radially to this bound: v <- v * (v_max / ||v||).
     * Units: Meters per second (m/s).
     * Validation: Must be strictly positive and finite. Range: [0.1, 10.0].
     * Default: 1.50 m/s.
     */
    double max_linear_velocity_m_s{1.50};

    /**
     * @brief Maximum allowable angular velocity magnitude |omega| in rover body frame.
     * Exceeding angular velocities are clamped: omega <- clamp(omega, -w_max, w_max).
     * Units: Radians per second (rad/s).
     * Validation: Must be strictly positive and finite. Range: [0.1, 5.0].
     * Default: 1.00 rad/s.
     */
    double max_angular_velocity_rad_s{1.00};

    /**
     * @brief Linear velocity deadband threshold.
     * If ||v|| < threshold, linear velocity is zeroed to prevent quiescent drift and noise amplification.
     * Units: Meters per second (m/s). Range: [0.0, 0.1].
     * Default: 1e-4 m/s.
     */
    double velocity_deadband_linear{1e-4};

    /**
     * @brief Angular velocity deadband threshold.
     * If |omega| < threshold, angular velocity is zeroed.
     * Units: Radians per second (rad/s). Range: [0.0, 0.1].
     * Default: 1e-4 rad/s.
     */
    double velocity_deadband_angular{1e-4};

    /**
     * @brief Maximum permitted prediction horizon tau for velocity extrapolation.
     * If packet age exceeds this horizon (age > max_prediction_horizon_s), extrapolation
     * is disabled and the node cleanly falls back to Zero-Order Hold (ZOH).
     * Units: Seconds (s). Range: [0.001, 1.0].
     * Default: 0.030 s (30 ms).
     */
    double max_prediction_horizon_s{0.030};

    /**
     * @brief Toggle for empirical delay-dependent adaptive damping schedule.
     * Schedule: D_i(tau) = D_base * min(max_multiplier, 1 + alpha_d * tau)
     * Rationale: Compensates negative damping / phase lag injected by delayed boundary coupling.
     * Classification: Empirical control heuristic (not an absolute Lyapunov convergence theorem).
     * Default: false.
     */
    bool enable_adaptive_damping{false};

    /**
     * @brief Damping growth rate alpha_d with respect to delay.
     * Units: Inverse seconds (s^-1, or 1/s). Dimensionless product: alpha_d * tau.
     * Validation: Must be non-negative and finite. Range: [0.0, 100.0].
     * Default: 15.0 s^-1.
     */
    double adaptive_damping_alpha{15.0};

    /**
     * @brief Ceiling multiplier for adaptive damping D_i(tau) / D_base.
     * Prevents extreme over-damping or integrator numerical stiffness at large latencies.
     * Units: Dimensionless ratio >= 1.0. Range: [1.0, 20.0].
     * Default: 5.0 (maximum 5x base damping).
     */
    double max_damping_multiplier{5.0};

    /**
     * @brief Validate that all parameter fields reside within admissible physical and numerical ranges.
     */
    bool is_valid() const noexcept {
        if (!std::isfinite(velocity_smoothing_factor) || velocity_smoothing_factor <= 0.0 || velocity_smoothing_factor > 1.0) {
            return false;
        }
        if (!std::isfinite(max_linear_velocity_m_s) || max_linear_velocity_m_s <= 0.0) {
            return false;
        }
        if (!std::isfinite(max_angular_velocity_rad_s) || max_angular_velocity_rad_s <= 0.0) {
            return false;
        }
        if (!std::isfinite(velocity_deadband_linear) || velocity_deadband_linear < 0.0) {
            return false;
        }
        if (!std::isfinite(velocity_deadband_angular) || velocity_deadband_angular < 0.0) {
            return false;
        }
        if (!std::isfinite(max_prediction_horizon_s) || max_prediction_horizon_s <= 0.0) {
            return false;
        }
        if (!std::isfinite(adaptive_damping_alpha) || adaptive_damping_alpha < 0.0) {
            return false;
        }
        if (!std::isfinite(max_damping_multiplier) || max_damping_multiplier < 1.0) {
            return false;
        }
        return true;
    }
};

/**
 * @brief Filtered state history for a single boundary neighbor node.
 */
struct FilteredBoundarySample {
    graph::NodeId node_id;
    uint32_t sender_rover_id{0};
    uint32_t target_rover_id{0};
    geometry::SE2 latest_pose;
    geometry::Twist2D raw_velocity;
    geometry::Twist2D filtered_velocity;
    uint64_t packet_timestamp_ns{0};
    uint64_t reception_timestamp_ns{0};
    uint32_t sequence_number{0};
    bool has_sample{false};
};

/**
 * @brief Discrete-time boundary-state filter and extrapolation engine for distributed CORD.
 *
 * Implements:
 * 1. Packet header, timestamp, sequence, and non-finite value validation.
 * 2. Stale/duplicate sequence rejection via RFC 1982 modular comparisons.
 * 3. Exponential moving average (EMA) low-pass smoothing on velocity twists.
 * 4. Radial linear velocity clamping and angular velocity saturation.
 * 5. Velocity deadbands to suppress quiescent sensor/numerical noise.
 * 6. Prediction-horizon gating and clean Zero-Order Hold (ZOH) fallback.
 * 7. Empirical delay-dependent adaptive damping schedule computation.
 */
class BoundaryStateFilter {
public:
    explicit BoundaryStateFilter(const BoundaryFilterOptions& options = BoundaryFilterOptions());

    const BoundaryFilterOptions& options() const noexcept { return options_; }
    void set_options(const BoundaryFilterOptions& options) noexcept { options_ = options; }

    /**
     * @brief Ingest and filter an incoming boundary state packet.
     *
     * Validates packet integrity, sequence ordering, and non-finite coordinates.
     * Applies velocity low-pass filtering, norm clamping, and deadband.
     * Stores reference pose strictly on SE(2) without interpolating raw angles.
     *
     * @param packet Received boundary state packet.
     * @param reception_time_ns Current local node simulation reception timestamp.
     * @return true if packet was valid, newer, and successfully processed; false if rejected.
     */
    bool process_packet(const BoundaryStatePacket& packet, uint64_t reception_time_ns);

    /**
     * @brief Compute the estimated pose and velocity for a neighbor node at query_time_ns.
     *
     * If prediction is disabled, age exceeds max_prediction_horizon_s, or motion exceeds bounds,
     * cleanly falls back to Zero-Order Hold (ZOH) with zero velocity.
     *
     * @param neighbor_id Remote neighbor node identifier.
     * @param query_time_ns Current simulation timestamp.
     * @param out_pose Destination for estimated SE(2) pose.
     * @param out_velocity Destination for estimated body-twist velocity.
     * @param out_is_fallback True if prediction fell back to ZOH, false if actively extrapolated.
     * @return true if a valid sample exists and estimate was generated, false otherwise.
     */
    bool get_estimated_state(
        const graph::NodeId& neighbor_id,
        uint64_t query_time_ns,
        geometry::SE2& out_pose,
        geometry::Twist2D& out_velocity,
        bool& out_is_fallback) const;

    /**
     * @brief Query the latest filtered sample record for a neighbor.
     */
    const FilteredBoundarySample* get_sample(const graph::NodeId& neighbor_id) const;

    /**
     * @brief Calculate the adaptive damping multiplier for a given observed boundary delay.
     *
     * Formula: min(max_multiplier, 1.0 + alpha_d * max(0.0, delay_s))
     *
     * @param delay_s Observed packet age/delay in seconds.
     * @return Damping multiplier in [1.0, max_damping_multiplier]. Returns 1.0 if adaptive damping is disabled.
     */
    double compute_damping_multiplier(double delay_s) const;

    /**
     * @brief Reset tracking state for a given neighbor node.
     */
    void reset(const graph::NodeId& neighbor_id);

    /**
     * @brief Reset all neighbor tracking states.
     */
    void clear();

    /**
     * @brief Telemetry: total valid packets ingested and filtered.
     */
    size_t accepted_packet_count() const noexcept { return accepted_packets_; }

    /**
     * @brief Telemetry: total packets rejected (corrupted, out-of-order, stale, or non-finite).
     */
    size_t rejected_packet_count() const noexcept { return rejected_packets_; }

    /**
     * @brief Telemetry: total prediction queries that fell back to ZOH.
     */
    size_t fallback_count() const noexcept { return fallback_count_; }

private:
    BoundaryFilterOptions options_;
    std::unordered_map<graph::NodeId, FilteredBoundarySample> samples_;
    mutable size_t accepted_packets_{0};
    mutable size_t rejected_packets_{0};
    mutable size_t fallback_count_{0};
};

} // namespace astromesh::optimization
