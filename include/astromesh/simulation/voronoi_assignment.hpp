#pragma once

#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace astromesh::simulation {

/**
 * @brief Represents a generator site for nearest-site 2D Voronoi spatial partitioning.
 */
struct VoronoiSite {
    uint16_t site_id{0};
    verification::Point2D position{0.0, 0.0};
    std::string label;

    bool is_valid() const noexcept {
        return site_id > 0 && position.is_valid();
    }
};

/**
 * @brief Outcome of a nearest-site Voronoi query for a 2D spatial coordinate.
 */
struct VoronoiAssignmentResult {
    bool valid{false};
    uint16_t assigned_site_id{0};
    double distance{0.0};
    bool is_equidistant_tie{false}; // Set to true if query point lies on a boundary between multiple sites
};

/**
 * @brief Nearest-site 2D Voronoi spatial partition assigner with deterministic tie-breaking.
 *
 * Mathematical Definition:
 * For a set of K generator sites S = {s_1, ..., s_K} in R^2, a query point q is assigned
 * to the Voronoi cell V(s_i) if:
 *   ||q - s_i||_2 < ||q - s_j||_2  for all j != i.
 *
 * Equidistant Boundary Policy:
 * If multiple sites achieve the exact minimum distance (within epsilon = 1e-9 meters),
 * the tie is broken deterministically by selecting the site with the smallest numerical site_id.
 */
class NearestSiteVoronoiAssignment {
public:
    NearestSiteVoronoiAssignment() = default;

    /**
     * @brief Constructs assignment engine with an explicit set of generator sites.
     * @throws std::invalid_argument if sites are empty, contain duplicates, or have invalid coordinates.
     */
    explicit NearestSiteVoronoiAssignment(const std::vector<VoronoiSite>& sites);

    bool is_valid() const noexcept { return !sites_.empty(); }
    size_t num_sites() const noexcept { return sites_.size(); }
    const std::vector<VoronoiSite>& sites() const noexcept { return sites_; }

    /**
     * @brief Queries which Voronoi site cell contains the given spatial point.
     * @param query_point 2D position in meters.
     * @return VoronoiAssignmentResult containing the assigned site ID and distance.
     */
    VoronoiAssignmentResult assign(const verification::Point2D& query_point) const;

private:
    std::vector<VoronoiSite> sites_;
};

} // namespace astromesh::simulation
