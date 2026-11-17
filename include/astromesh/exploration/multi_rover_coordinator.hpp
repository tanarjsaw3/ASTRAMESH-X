#pragma once

#include "astromesh/exploration/frontier.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace astromesh::exploration {

#pragma pack(push, 1)
/**
 * @brief Wire-packed decentralized frontier claim broadcast (44 bytes).
 */
struct FrontierClaim {
    uint64_t frontier_id{0};
    uint16_t rover_id{0};
    uint16_t reserved{0};
    double target_x{0.0};
    double target_y{0.0};
    double utility_score{0.0};
    uint32_t lease_ttl_steps{30};
    uint32_t epoch{1};

    void serialize(std::vector<uint8_t>& buf) const;
    static bool deserialize(const uint8_t* data, size_t len, FrontierClaim& out);
};
#pragma pack(pop)

static_assert(sizeof(FrontierClaim) == 44, "FrontierClaim must be exactly 44 bytes packed");

/**
 * @brief Coordinates multi-rover frontier assignments, leases, and deterministic conflict resolution without a central master.
 */
class MultiRoverExplorationCoordinator {
public:
    explicit MultiRoverExplorationCoordinator(uint16_t local_rover_id, double separation_radius = 8.0);

    /**
     * @brief Creates or renews a local lease claim on a target frontier.
     */
    FrontierClaim create_local_claim(
        uint64_t frontier_id,
        const verification::Point2D& target_position,
        double utility_score,
        uint64_t current_step,
        uint32_t ttl_steps = 30);

    /**
     * @brief Ingests an incoming claim from a peer rover.
     * @return true if claim was accepted into ledger; false if stale or rejected.
     */
    bool process_remote_claim(const FrontierClaim& claim, uint64_t current_step);

    /**
     * @brief Determines whether a remote candidate dominates an existing claim under deterministic tie-breaking.
     * Dominance rules:
     * 1. Higher utility score.
     * 2. Lower reservation epoch.
     * 3. Lower numerical rover ID.
     */
    static bool candidate_dominates(
        double cand_util, uint32_t cand_epoch, uint16_t cand_rover,
        double exist_util, uint32_t exist_epoch, uint16_t exist_rover) noexcept;

    /**
     * @brief Checks if a proposed target coordinate conflicts with an active peer claim within separation radius.
     */
    bool has_peer_conflict(
        const verification::Point2D& target,
        double utility,
        uint64_t current_step,
        uint16_t* conflicting_peer = nullptr) const;

    /**
     * @brief Prunes expired leases from the claim registry.
     */
    size_t prune_expired_claims(uint64_t current_step);

    /**
     * @brief Retrieves all active peer target positions to penalize redundancy during utility scoring.
     */
    std::vector<verification::Point2D> active_peer_targets(uint64_t current_step) const;

    uint16_t local_rover_id() const noexcept { return m_local_rover_id; }
    const std::map<uint64_t, FrontierClaim>& active_claims() const noexcept { return m_claims; }

    bool is_local_claim_active(uint64_t current_step) const noexcept;
    const FrontierClaim& local_claim() const noexcept { return m_local_claim; }

    void clear() noexcept;

private:
    uint16_t m_local_rover_id{0};
    double m_separation_radius{8.0};
    uint32_t m_current_epoch{1};

    FrontierClaim m_local_claim;
    uint64_t m_local_claim_expiration{0};

    // FrontierID -> Active Claim
    std::map<uint64_t, FrontierClaim> m_claims;
    std::map<uint64_t, uint64_t> m_claim_expirations;
};

} // namespace astromesh::exploration
