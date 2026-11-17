#include "astromesh/simulation/failure_injector.hpp"

#include <algorithm>

namespace astromesh::simulation {

FailureInjector::FailureInjector(uint64_t seed)
    : m_seed(seed), m_rng(seed) {
}

void FailureInjector::set_gilbert_elliott_params(GilbertElliottParams params) {
    m_ge_params = params;
}

std::pair<uint16_t, uint16_t> FailureInjector::canonical_link(uint16_t a, uint16_t b) const noexcept {
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

void FailureInjector::schedule_partition(uint16_t rover_a, uint16_t rover_b, uint64_t start_ms, uint64_t end_ms) {
    m_partitions.push_back({rover_a, rover_b, start_ms, end_ms});
}

void FailureInjector::schedule_node_crash(uint16_t rover_id, uint64_t crash_ms, uint64_t restart_ms) {
    m_crashes.push_back({rover_id, crash_ms, restart_ms});
}

bool FailureInjector::is_link_partitioned(uint16_t rover_a, uint16_t rover_b, uint64_t current_time_ms) const noexcept {
    auto target = canonical_link(rover_a, rover_b);
    for (const auto& p : m_partitions) {
        if (canonical_link(p.rover_a, p.rover_b) == target) {
            if (current_time_ms >= p.start_time_ms && current_time_ms < p.end_time_ms) {
                return true;
            }
        }
    }
    return false;
}

bool FailureInjector::is_node_crashed(uint16_t rover_id, uint64_t current_time_ms) const noexcept {
    for (const auto& c : m_crashes) {
        if (c.rover_id == rover_id) {
            if (current_time_ms >= c.crash_time_ms && current_time_ms < c.restart_time_ms) {
                return true;
            }
        }
    }
    return false;
}

bool FailureInjector::should_drop_packet(uint16_t from_rover, uint16_t to_rover, uint64_t current_time_ms) {
    // 1. Check if either endpoint is crashed
    if (is_node_crashed(from_rover, current_time_ms) || is_node_crashed(to_rover, current_time_ms)) {
        return true;
    }

    // 2. Check if link is severed by scheduled network partition
    if (is_link_partitioned(from_rover, to_rover, current_time_ms)) {
        return true;
    }

    // 3. Gilbert-Elliott Burst Loss Simulation
    auto link = canonical_link(from_rover, to_rover);
    auto it = m_link_states.find(link);
    if (it == m_link_states.end()) {
        m_link_states[link] = ChannelState::Good;
        it = m_link_states.find(link);
    }

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // Channel state transition
    if (it->second == ChannelState::Good) {
        if (dist(m_rng) < m_ge_params.p_gb) {
            it->second = ChannelState::Bad;
        }
    } else {
        if (dist(m_rng) < m_ge_params.p_bg) {
            it->second = ChannelState::Good;
        }
    }

    // Packet drop probability based on current state
    double drop_prob = (it->second == ChannelState::Good) ? m_ge_params.p_g_loss : m_ge_params.p_b_loss;
    return dist(m_rng) < drop_prob;
}

double FailureInjector::sample_latency_ms(double mean_ms, double stddev_ms) {
    std::normal_distribution<double> dist(mean_ms, stddev_ms);
    return std::max(0.1, dist(m_rng));
}

void FailureInjector::reset() {
    m_rng.seed(m_seed);
    m_partitions.clear();
    m_crashes.clear();
    m_link_states.clear();
}

} // namespace astromesh::simulation

