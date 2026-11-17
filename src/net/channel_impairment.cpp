#include "astromesh/net/channel_impairment.hpp"

#include <algorithm>

namespace astromesh::net {

ChannelImpairment::ChannelImpairment(const ChannelImpairmentConfig& config)
    : m_config(config), m_rng_state(config.seed != 0 ? config.seed : 42) {}

void ChannelImpairment::set_config(const ChannelImpairmentConfig& config) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config = config;
    if (config.seed != 0) {
        m_rng_state = config.seed;
    }
}

const ChannelImpairmentConfig& ChannelImpairment::get_config() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_config;
}

void ChannelImpairment::set_loss_rate(double loss_rate) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config.packet_loss_rate = std::clamp(loss_rate, 0.0, 1.0);
}

void ChannelImpairment::set_delay_ns(uint64_t delay_ns) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config.one_way_delay_ns = delay_ns;
}

void ChannelImpairment::set_global_blackout(bool blackout) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config.global_blackout = blackout;
}

void ChannelImpairment::set_link_state(uint16_t rover_a, uint16_t rover_b, bool connected) {
    if (rover_a == rover_b) return;
    const uint16_t lo = std::min(rover_a, rover_b);
    const uint16_t hi = std::max(rover_a, rover_b);
    const uint32_t key = (static_cast<uint32_t>(lo) << 16) | static_cast<uint32_t>(hi);

    std::lock_guard<std::mutex> lock(m_mutex);
    m_bilateral_links[key] = connected;
}

bool ChannelImpairment::is_link_connected(uint16_t rover_a, uint16_t rover_b) const {
    if (rover_a == rover_b) return true;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_config.global_blackout) return false;

    const uint16_t lo = std::min(rover_a, rover_b);
    const uint16_t hi = std::max(rover_a, rover_b);
    const uint32_t key = (static_cast<uint32_t>(lo) << 16) | static_cast<uint32_t>(hi);

    auto it = m_bilateral_links.find(key);
    if (it != m_bilateral_links.end()) {
        return it->second;
    }
    return true; // Default connected if not explicitly partitioned
}

uint64_t ChannelImpairment::next_random() {
    // 64-bit SplitMix PRNG
    m_rng_state += 0x9e3779b97f4a7c15ULL;
    uint64_t z = m_rng_state;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

double ChannelImpairment::next_uniform() {
    return static_cast<double>(next_random() >> 11) * (1.0 / 9007199254740992.0); // 2^53
}

bool ChannelImpairment::process_outgoing(
    const uint8_t* data, size_t size,
    const Endpoint& dest,
    uint16_t from_rover, uint16_t to_rover,
    uint64_t now_ns,
    bool& out_immediate) {

    std::lock_guard<std::mutex> lock(m_mutex);
    m_stats.packets_submitted++;
    m_stats.bytes_submitted += size;
    out_immediate = false;

    // 1. Check global or bilateral link blackout
    if (m_config.global_blackout) {
        m_stats.packets_dropped_blackout++;
        return false;
    }

    if (to_rover != 0 && from_rover != 0) {
        const uint16_t lo = std::min(from_rover, to_rover);
        const uint16_t hi = std::max(from_rover, to_rover);
        const uint32_t key = (static_cast<uint32_t>(lo) << 16) | static_cast<uint32_t>(hi);
        auto it = m_bilateral_links.find(key);
        if (it != m_bilateral_links.end() && !it->second) {
            m_stats.packets_dropped_blackout++;
            return false;
        }
    }

    // 2. Check stochastic packet loss
    if (m_config.packet_loss_rate > 0.0) {
        const double sample = next_uniform();
        if (sample < m_config.packet_loss_rate) {
            m_stats.packets_dropped_loss++;
            return false;
        }
    }

    // 3. Propagation latency handling
    if (m_config.one_way_delay_ns > 0) {
        DelayedPacket delayed;
        delayed.data.assign(data, data + size);
        delayed.destination = dest;
        delayed.release_time_ns = now_ns + m_config.one_way_delay_ns;
        m_delayed_queue.push_back(std::move(delayed));
        m_stats.packets_delayed++;
        return true;
    }

    // Direct immediate transmission
    out_immediate = true;
    m_stats.packets_passed++;
    m_stats.bytes_passed += size;
    return true;
}

std::vector<ChannelImpairment::DelayedPacket> ChannelImpairment::release_due_packets(uint64_t now_ns) {
    std::vector<DelayedPacket> ready;
    std::lock_guard<std::mutex> lock(m_mutex);

    while (!m_delayed_queue.empty()) {
        if (now_ns >= m_delayed_queue.front().release_time_ns) {
            ready.push_back(std::move(m_delayed_queue.front()));
            m_delayed_queue.pop_front();
            m_stats.packets_passed++;
            m_stats.bytes_passed += ready.back().data.size();
        } else {
            break; // Since delays are monotonic per packet arrival order
        }
    }

    return ready;
}

size_t ChannelImpairment::pending_delayed_count() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_delayed_queue.size();
}

void ChannelImpairment::reset(uint64_t seed) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rng_state = seed != 0 ? seed : 42;
    m_delayed_queue.clear();
    m_bilateral_links.clear();
    m_stats = ChannelImpairmentStats{};
}

ChannelImpairmentStats ChannelImpairment::get_stats() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stats;
}

} // namespace astromesh::net

