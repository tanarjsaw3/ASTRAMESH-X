#include "astromesh/memory/memory_budget_controller.hpp"

namespace astromesh::memory {

MemoryPressureLevel MemoryBudgetController::evaluate_pressure(
    size_t current_usage_bytes,
    size_t indispensable_bytes) noexcept {

    m_metrics.total_evaluations++;

    size_t budget = m_config.per_rover_budget_bytes;
    if (budget == 0) {
        m_current_pressure = MemoryPressureLevel::Normal;
        m_budget_infeasible = false;
        return m_current_pressure;
    }

    // Check if indispensable state alone exceeds budget or leaves no headroom
    size_t usable_budget = (budget > static_cast<size_t>(budget * m_config.reserved_critical_ratio))
                               ? budget - static_cast<size_t>(budget * m_config.reserved_critical_ratio)
                               : budget;

    if (indispensable_bytes > usable_budget) {
        m_budget_infeasible = true;
        m_metrics.infeasible_budget_events++;
    } else {
        m_budget_infeasible = false;
    }

    double ratio = static_cast<double>(current_usage_bytes) / static_cast<double>(budget);

    // Apply pressure state transitions with hysteresis
    switch (m_current_pressure) {
        case MemoryPressureLevel::Normal:
            if (ratio >= m_config.critical_threshold_ratio) {
                m_current_pressure = MemoryPressureLevel::Critical;
                m_metrics.critical_events++;
                m_metrics.compactions_triggered++;
            } else if (ratio >= m_config.warning_threshold_ratio) {
                m_current_pressure = MemoryPressureLevel::Warning;
                m_metrics.warning_events++;
            }
            break;

        case MemoryPressureLevel::Warning:
            if (ratio >= m_config.critical_threshold_ratio) {
                m_current_pressure = MemoryPressureLevel::Critical;
                m_metrics.critical_events++;
                m_metrics.compactions_triggered++;
            } else if (ratio < m_config.recovery_threshold_ratio) {
                // Must fall below recovery threshold (e.g. 65%) to return to Normal
                m_current_pressure = MemoryPressureLevel::Normal;
            }
            break;

        case MemoryPressureLevel::Critical:
            if (ratio < m_config.recovery_threshold_ratio) {
                m_current_pressure = MemoryPressureLevel::Normal;
            } else if (ratio < m_config.critical_threshold_ratio) {
                m_current_pressure = MemoryPressureLevel::Warning;
            }
            break;
    }

    return m_current_pressure;
}

} // namespace astromesh::memory

