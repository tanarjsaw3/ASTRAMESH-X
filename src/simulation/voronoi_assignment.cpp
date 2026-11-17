#include "astromesh/simulation/voronoi_assignment.hpp"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace astromesh::simulation {

NearestSiteVoronoiAssignment::NearestSiteVoronoiAssignment(const std::vector<VoronoiSite>& sites)
    : sites_(sites) {
    if (sites_.empty()) {
        throw std::invalid_argument("NearestSiteVoronoiAssignment: at least one generator site is required.");
    }

    std::set<uint16_t> seen_ids;
    for (const auto& site : sites_) {
        if (!site.is_valid()) {
            throw std::invalid_argument("NearestSiteVoronoiAssignment: site has invalid ID or non-finite position.");
        }
        if (!seen_ids.insert(site.site_id).second) {
            throw std::invalid_argument("NearestSiteVoronoiAssignment: duplicate site ID detected.");
        }
    }
}

VoronoiAssignmentResult NearestSiteVoronoiAssignment::assign(const verification::Point2D& query_point) const {
    VoronoiAssignmentResult res;
    if (sites_.empty() || !query_point.is_valid()) {
        res.valid = false;
        return res;
    }

    constexpr double EPS = 1e-9;
    double min_dist = std::numeric_limits<double>::infinity();
    uint16_t best_id = 0;
    bool tie_found = false;

    for (const auto& site : sites_) {
        const double dx = query_point.x - site.position.x;
        const double dy = query_point.y - site.position.y;
        const double d = std::hypot(dx, dy);

        if (d < min_dist - EPS) {
            min_dist = d;
            best_id = site.site_id;
            tie_found = false;
        } else if (std::abs(d - min_dist) <= EPS) {
            // Equidistant boundary tie: break tie deterministically by smallest site_id
            tie_found = true;
            if (site.site_id < best_id) {
                best_id = site.site_id;
            }
        }
    }

    res.valid = true;
    res.assigned_site_id = best_id;
    res.distance = min_dist;
    res.is_equidistant_tie = tie_found;

    return res;
}

} // namespace astromesh::simulation
