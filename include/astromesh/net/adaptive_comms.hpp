#pragma once

#include <cstdint>
#include <algorithm>
#include <chrono>

namespace astromesh::net {

enum class NetworkHealthRegime : uint8_t {
    Nominal = 0,    ///< Low loss, low queue: high transmission rate, standard batches.
    Congested = 1,  ///< Moderate loss / queue: backoff rate, smaller batches, larger IBLT cells.
    Impaired = 2    ///< Severe loss / near-partition: aggressive backoff, emergency heartbeats only.
};

struct AdaptiveCommsConfig {
    double ewma_alpha{0.20};                ///< Smoothing factor for loss and RTT.
    double loss_congested_thresh{0.10};     ///< Loss >= 10% triggers Congested.
    double loss_impaired_thresh{0.35};      ///< Loss >= 35% triggers Impaired.
    double queue_congested_thresh{0.50};    ///< Queue >= 50% capacity triggers Congested.
    double queue_impaired_thresh{0.80};     ///< Queue >= 80% capacity triggers Impaired.

    // Timing intervals (milliseconds)
    uint32_t interval_nominal_ms{20};       ///< 50 Hz
    uint32_t interval_congested_ms{50};     ///< 20 Hz
    uint32_t interval_impaired_ms{200};     ///< 5 Hz

    // Batch sizes (updates per packet)
    uint32_t batch_nominal{8};
    uint32_t batch_congested{4};
    uint32_t batch_impaired{2};

    // IBLT cell sizing
    size_t iblt_cells_nominal{32};
    size_t iblt_cells_congested{64};
    size_t iblt_cells_impaired{64};
};

/**
 * @brief Adaptive Multi-Hop Communication Policy Controller.
 * Evaluates channel loss, queue pressure, and round-trip delay to dynamically
 * regulate message pacing, batching, and IBLT dimensioning.
 */
class AdaptiveCommsController {
public:
    explicit AdaptiveCommsController(AdaptiveCommsConfig config = AdaptiveCommsConfig{});

    /**
     * @brief Ingests observed sample of link loss, RTT, and local queue fullness.
     */
    void observe_sample(double instantaneous_loss, double rtt_ms, double queue_occupancy_fraction);

    NetworkHealthRegime regime() const noexcept { return m_regime; }
    double smoothed_loss() const noexcept { return m_smoothed_loss; }
    double smoothed_rtt_ms() const noexcept { return m_smoothed_rtt_ms; }
    double smoothed_queue_fraction() const noexcept { return m_smoothed_queue; }

    uint32_t current_interval_ms() const noexcept;
    uint32_t current_batch_size() const noexcept;
    size_t current_iblt_cells() const noexcept;

    const AdaptiveCommsConfig& config() const noexcept { return m_config; }

private:
    void update_regime();

    AdaptiveCommsConfig m_config;
    NetworkHealthRegime m_regime{NetworkHealthRegime::Nominal};

    double m_smoothed_loss{0.0};
    double m_smoothed_rtt_ms{10.0};
    double m_smoothed_queue{0.0};
    bool m_initialized{false};
};

} // namespace astromesh::net

