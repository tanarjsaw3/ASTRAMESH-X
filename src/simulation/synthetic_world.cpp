#include "astromesh/simulation/synthetic_world.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace astromesh::simulation {

namespace {

// Deterministic 64-bit SplitMix64 / FNV-1a hash for landmark descriptors
uint64_t hash_landmark_descriptor(uint32_t id, double x, double y, uint64_t seed) noexcept {
    uint64_t h = seed ^ 0xcbf29ce484222325ULL;
    h ^= static_cast<uint64_t>(id);
    h *= 0x100000001b3ULL;

    union { double d; uint64_t u; } ux{x}, uy{y};
    h ^= ux.u;
    h *= 0x100000001b3ULL;
    h ^= uy.u;
    h *= 0x100000001b3ULL;

    // SplitMix64 finalizer
    h ^= h >> 30;
    h *= 0xbf58476d1ce4e5b9ULL;
    h ^= h >> 27;
    h *= 0x94d049bb133111ebULL;
    h ^= h >> 31;
    return h == 0 ? 1 : h; // Avoid 0
}

} // namespace

bool WorldConfig::is_valid() const noexcept {
    if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
        !std::isfinite(min_y) || !std::isfinite(max_y)) {
        return false;
    }
    if (min_x >= max_x || min_y >= max_y) {
        return false;
    }
    if (!std::isfinite(sensor_range) || sensor_range <= 0.0) {
        return false;
    }
    if (!std::isfinite(field_of_view_rad) || field_of_view_rad <= 0.0 || field_of_view_rad > 2.0 * M_PI + 1e-9) {
        return false;
    }
    return true;
}

SyntheticWorld2D::SyntheticWorld2D(const WorldConfig& config)
    : config_(config) {
    if (!config_.is_valid()) {
        throw std::invalid_argument("SyntheticWorld2D: invalid WorldConfig specified.");
    }
    generate_landmarks();
}

void SyntheticWorld2D::generate_landmarks() {
    landmarks_.clear();
    landmarks_.reserve(config_.num_landmarks);

    std::mt19937_64 rng(config_.seed);
    std::uniform_real_distribution<double> dist_x(config_.min_x, config_.max_x);
    std::uniform_real_distribution<double> dist_y(config_.min_y, config_.max_y);

    for (size_t i = 1; i <= config_.num_landmarks; ++i) {
        WorldLandmark lm;
        lm.id = static_cast<uint32_t>(i);
        lm.position.x = dist_x(rng);
        lm.position.y = dist_y(rng);

        const uint64_t desc_hash = hash_landmark_descriptor(lm.id, lm.position.x, lm.position.y, config_.seed);
        lm.key = core::LandmarkKey(1, 0, lm.id, desc_hash);

        landmarks_.push_back(lm);
    }
}

std::vector<WorldLandmark> SyntheticWorld2D::query_visible_landmarks(const geometry::SE2& rover_pose) const {
    if (!rover_pose.is_valid()) {
        return {};
    }

    std::vector<WorldLandmark> visible;
    const double rx = rover_pose.x();
    const double ry = rover_pose.y();
    const double rtheta = rover_pose.theta();

    const double r_sq = config_.sensor_range * config_.sensor_range;
    const bool is_omnidirectional = (config_.field_of_view_rad >= 2.0 * M_PI - 1e-9);
    const double half_fov = config_.field_of_view_rad * 0.5;

    for (const auto& lm : landmarks_) {
        const double dx = lm.position.x - rx;
        const double dy = lm.position.y - ry;
        const double dist_sq = dx * dx + dy * dy;

        // Radial range check
        if (dist_sq > r_sq) {
            continue;
        }

        // Angular FOV check
        if (!is_omnidirectional) {
            const double absolute_angle = std::atan2(dy, dx);
            const double relative_bearing = graph::normalize_angle(absolute_angle - rtheta);
            if (std::abs(relative_bearing) > half_fov) {
                continue;
            }
        }

        visible.push_back(lm);
    }

    // Deterministic ordering by landmark ID
    std::sort(visible.begin(), visible.end(), [](const WorldLandmark& a, const WorldLandmark& b) {
        return a.id < b.id;
    });

    return visible;
}

const WorldLandmark* SyntheticWorld2D::find_landmark(uint32_t landmark_id) const noexcept {
    for (const auto& lm : landmarks_) {
        if (lm.id == landmark_id) {
            return &lm;
        }
    }
    return nullptr;
}

} // namespace astromesh::simulation
