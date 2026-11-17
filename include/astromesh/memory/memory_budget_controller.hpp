#ifndef ASTROMESH_MEMORY_BUDGET_CONTROLLER_HPP
#define ASTROMESH_MEMORY_BUDGET_CONTROLLER_HPP

#include <cstddef>
#include <cstdint>

namespace astromesh::memory {

/**
 * @brief Discrete memory pressure tiers.
 */
enum class MemoryPressureLevel : uint8_t {
    Normal = 0,   ///< Usage < warning threshold
    Warning = 1,  ///< Warning threshold <= Usage < critical threshold
    Critical = 2  ///< Usage >= critical threshold
};

/**
 * @brief Configuration parameters for memory budget controller.
 */
struct MemoryBudgetConfig {
    size_t per_rover_budget_bytes{256 * 1024};    ///< Default 256 KB per rover
    size_t global_swarm_budget_bytes{2048 * 1024}; ///< Default 2 MB for swarm
    double warning_threshold_ratio{0.75};         ///< 75%
    double critical_threshold_ratio{0.90};        ///< 90%
    double recovery_threshold_ratio{0.65};        ///< 65% for hysteresis exit
    double reserved_critical_ratio{0.20};         ///< 20% reserved for indispensable state
};

/**
 * @brief Runtime tracking metrics for budget events.
 */
struct MemoryBudgetMetrics {
    size_t total_evaluations{0};
    size_t warning_events{0};
    size_t critical_events{0};
    size_t compactions_triggered{0};
    size_t infeasible_budget_events{0};
};

/**
 * @brief Manages memory pressure states, hysteresis, and compaction trigger criteria.
 */
class MemoryBudgetController {
public:
    explicit MemoryBudgetController(const MemoryBudgetConfig& config = {}) noexcept
        : m_config(config) {}

    /**
     * @brief Evaluates current memory usage against budget with hysteresis.
     * @param current_usage_bytes Current total memory usage in bytes.
     * @param indispensable_bytes Unprunable memory footprint (frontier nodes, anchors, loop endpoints).
     * @return Resulting MemoryPressureLevel.
     */
    MemoryPressureLevel evaluate_pressure(size_t current_usage_bytes, size_t indispensable_bytes) noexcept;

    MemoryPressureLevel current_pressure() const noexcept { return m_current_pressure; }
    bool is_budget_infeasible() const noexcept { return m_budget_infeasible; }

    double budget_utilization_ratio(size_t current_usage_bytes) const noexcept {
        if (m_config.per_rover_budget_bytes == 0) return 0.0;
        return static_cast<double>(current_usage_bytes) / static_cast<double>(m_config.per_rover_budget_bytes);
    }

    const MemoryBudgetConfig& config() const noexcept { return m_config; }
    void set_config(const MemoryBudgetConfig& config) noexcept { m_config = config; }

    const MemoryBudgetMetrics& metrics() const noexcept { return m_metrics; }
    void reset_metrics() noexcept { m_metrics = {}; }

private:
    MemoryBudgetConfig m_config;
    MemoryPressureLevel m_current_pressure{MemoryPressureLevel::Normal};
    bool m_budget_infeasible{false};
    MemoryBudgetMetrics m_metrics;
};

} // namespace astromesh::memory

#endif // ASTROMESH_MEMORY_BUDGET_CONTROLLER_HPP

