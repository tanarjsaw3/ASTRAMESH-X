#include "astromesh/net/adaptive_comms.hpp"

namespace astromesh::net {

AdaptiveCommsController::AdaptiveCommsController(AdaptiveCommsConfig config)
    : m_config(config) {
}

void AdaptiveCommsController::observe_sample(
    double instantaneous_loss,
    double rtt_ms,
    double queue_occupancy_fraction) {

    instantaneous_loss = std::clamp(instantaneous_loss, 0.0, 1.0);
    queue_occupancy_fraction = std::clamp(queue_occupancy_fraction, 0.0, 1.0);
    if (rtt_ms < 0.0) rtt_ms = 0.0;

    if (!m_initialized) {
        m_smoothed_loss = instantaneous_loss;
        m_smoothed_rtt_ms = rtt_ms;
        m_smoothed_queue = queue_occupancy_fraction;
        m_initialized = true;
    } else {
        double a = m_config.ewma_alpha;
        m_smoothed_loss = (1.0 - a) * m_smoothed_loss + a * instantaneous_loss;
        m_smoothed_rtt_ms = (1.0 - a) * m_smoothed_rtt_ms + a * rtt_ms;
        m_smoothed_queue = (1.0 - a) * m_smoothed_queue + a * queue_occupancy_fraction;
    }

    update_regime();
}

void AdaptiveCommsController::update_regime() {
    if (m_smoothed_loss >= m_config.loss_impaired_thresh ||
        m_smoothed_queue >= m_config.queue_impaired_thresh) {
        m_regime = NetworkHealthRegime::Impaired;
    } else if (m_smoothed_loss >= m_config.loss_congested_thresh ||
               m_smoothed_queue >= m_config.queue_congested_thresh) {
        m_regime = NetworkHealthRegime::Congested;
    } else {
        m_regime = NetworkHealthRegime::Nominal;
    }
}

uint32_t AdaptiveCommsController::current_interval_ms() const noexcept {
    switch (m_regime) {
        case NetworkHealthRegime::Impaired:  return m_config.interval_impaired_ms;
        case NetworkHealthRegime::Congested: return m_config.interval_congested_ms;
        case NetworkHealthRegime::Nominal:
        default:                             return m_config.interval_nominal_ms;
    }
}

uint32_t AdaptiveCommsController::current_batch_size() const noexcept {
    switch (m_regime) {
        case NetworkHealthRegime::Impaired:  return m_config.batch_impaired;
        case NetworkHealthRegime::Congested: return m_config.batch_congested;
        case NetworkHealthRegime::Nominal:
        default:                             return m_config.batch_nominal;
    }
}

size_t AdaptiveCommsController::current_iblt_cells() const noexcept {
    switch (m_regime) {
        case NetworkHealthRegime::Impaired:  return m_config.iblt_cells_impaired;
        case NetworkHealthRegime::Congested: return m_config.iblt_cells_congested;
        case NetworkHealthRegime::Nominal:
        default:                             return m_config.iblt_cells_nominal;
    }
}

} // namespace astromesh::net

