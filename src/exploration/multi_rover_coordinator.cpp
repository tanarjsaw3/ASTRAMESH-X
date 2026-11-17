#include "astromesh/exploration/multi_rover_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace astromesh::exploration {

void FrontierClaim::serialize(std::vector<uint8_t>& buf) const {
    size_t prev_size = buf.size();
    buf.resize(prev_size + sizeof(FrontierClaim));
    std::memcpy(buf.data() + prev_size, this, sizeof(FrontierClaim));
}

bool FrontierClaim::deserialize(const uint8_t* data, size_t len, FrontierClaim& out) {
    if (!data || len < sizeof(FrontierClaim)) {
        return false;
    }
    std::memcpy(&out, data, sizeof(FrontierClaim));
    return out.rover_id > 0 && out.frontier_id > 0;
}

MultiRoverExplorationCoordinator::MultiRoverExplorationCoordinator(
    uint16_t local_rover_id, double separation_radius)
    : m_local_rover_id(local_rover_id), m_separation_radius(separation_radius) {}

bool MultiRoverExplorationCoordinator::candidate_dominates(
    double cand_util, uint32_t cand_epoch, uint16_t cand_rover,
    double exist_util, uint32_t exist_epoch, uint16_t exist_rover) noexcept {

    // Rule 1: Higher utility score strictly dominates
    constexpr double EPS = 1e-4;
    if (cand_util > exist_util + EPS) return true;
    if (cand_util < exist_util - EPS) return false;

    // Rule 2: Earlier reservation epoch dominates if utilities are indistinguishable
    if (cand_epoch < exist_epoch) return true;
    if (cand_epoch > exist_epoch) return false;

    // Rule 3: Deterministic tie-breaking by lowest numerical rover ID
    return cand_rover < exist_rover;
}

FrontierClaim MultiRoverExplorationCoordinator::create_local_claim(
    uint64_t frontier_id,
    const verification::Point2D& target_position,
    double utility_score,
    uint64_t current_step,
    uint32_t ttl_steps) {

    FrontierClaim claim;
    claim.frontier_id = frontier_id;
    claim.rover_id = m_local_rover_id;
    claim.reserved = 0;
    claim.target_x = target_position.x;
    claim.target_y = target_position.y;
    claim.utility_score = utility_score;
    claim.lease_ttl_steps = ttl_steps;
    claim.epoch = m_current_epoch++;

    m_local_claim = claim;
    m_local_claim_expiration = current_step + ttl_steps;

    m_claims[frontier_id] = claim;
    m_claim_expirations[frontier_id] = m_local_claim_expiration;

    return claim;
}

bool MultiRoverExplorationCoordinator::process_remote_claim(
    const FrontierClaim& claim, uint64_t current_step) {

    if (claim.rover_id == 0 || claim.rover_id == m_local_rover_id) {
        return false;
    }

    uint64_t new_exp = current_step + claim.lease_ttl_steps;
    auto it = m_claims.find(claim.frontier_id);

    if (it != m_claims.end()) {
        const auto& existing = it->second;
        bool dominates = candidate_dominates(
            claim.utility_score, claim.epoch, claim.rover_id,
            existing.utility_score, existing.epoch, existing.rover_id);

        if (!dominates) {
            return false; // Stale or lower priority claim rejected
        }
    }

    // Insert or update winning claim
    m_claims[claim.frontier_id] = claim;
    m_claim_expirations[claim.frontier_id] = new_exp;

    // Check if this overrides our own active local claim
    if (m_local_claim.frontier_id == claim.frontier_id && claim.rover_id != m_local_rover_id) {
        // Yield local claim to dominant peer
        m_local_claim = FrontierClaim{};
        m_local_claim_expiration = 0;
    }

    return true;
}

bool MultiRoverExplorationCoordinator::has_peer_conflict(
    const verification::Point2D& target,
    double utility,
    uint64_t current_step,
    uint16_t* conflicting_peer) const {

    for (const auto& [fid, claim] : m_claims) {
        if (claim.rover_id == m_local_rover_id) continue;
        if (current_step >= m_claim_expirations.at(fid)) continue;

        double d = std::hypot(target.x - claim.target_x, target.y - claim.target_y);
        if (d < m_separation_radius) {
            // Check if peer claim dominates our candidate
            bool peer_dominates = candidate_dominates(
                claim.utility_score, claim.epoch, claim.rover_id,
                utility, m_current_epoch, m_local_rover_id);

            if (peer_dominates) {
                if (conflicting_peer) *conflicting_peer = claim.rover_id;
                return true;
            }
        }
    }

    return false;
}

size_t MultiRoverExplorationCoordinator::prune_expired_claims(uint64_t current_step) {
    size_t pruned = 0;
    std::vector<uint64_t> to_remove;

    for (const auto& [fid, exp] : m_claim_expirations) {
        if (current_step >= exp) {
            to_remove.push_back(fid);
        }
    }

    for (uint64_t fid : to_remove) {
        m_claims.erase(fid);
        m_claim_expirations.erase(fid);
        pruned++;
    }

    if (current_step >= m_local_claim_expiration) {
        m_local_claim = FrontierClaim{};
        m_local_claim_expiration = 0;
    }

    return pruned;
}

std::vector<verification::Point2D> MultiRoverExplorationCoordinator::active_peer_targets(
    uint64_t current_step) const {

    std::vector<verification::Point2D> targets;
    for (const auto& [fid, claim] : m_claims) {
        if (claim.rover_id != m_local_rover_id && current_step < m_claim_expirations.at(fid)) {
            targets.push_back(verification::Point2D(claim.target_x, claim.target_y));
        }
    }
    return targets;
}

bool MultiRoverExplorationCoordinator::is_local_claim_active(uint64_t current_step) const noexcept {
    return m_local_claim.frontier_id > 0 && current_step < m_local_claim_expiration;
}

void MultiRoverExplorationCoordinator::clear() noexcept {
    m_claims.clear();
    m_claim_expirations.clear();
    m_local_claim = FrontierClaim{};
    m_local_claim_expiration = 0;
    m_current_epoch = 1;
}

} // namespace astromesh::exploration

