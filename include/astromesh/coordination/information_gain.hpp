#pragma once

#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <vector>

namespace astromesh::coordination {

struct InformationGainWeights {
    double alpha_drift{1.0};       // Drift reduction scaling
    double alpha_dispersion{1.5};  // Spatial scatter scaling
    double alpha_topological{2.0}; // Bridge value scaling
    double alpha_redundancy{1.0};  // Close-edge penalty
};

struct InformationGainResult {
    double total_score{0.0};
    double drift_component{0.0};
    double dispersion_component{0.0};
    double topological_component{0.0};
    double redundancy_penalty{0.0};
    bool is_degenerate{false};
};

/**
 * @brief Evaluates decentralized information gain proxy metrics for prospective loop closures.
 * Enforces verifiable geometric dispersion, accumulated trajectory drift reduction, and topological bridge value.
 */
class InformationGainEvaluator {
public:
    explicit InformationGainEvaluator(InformationGainWeights weights = {})
        : m_weights(weights) {}

    const InformationGainWeights& weights() const noexcept { return m_weights; }
    void set_weights(const InformationGainWeights& w) noexcept { m_weights = w; }

    /**
     * @brief Computes 2D landmark dispersion determinant: det(1/N * sum (p - p_bar)(p - p_bar)^T).
     */
    static double compute_geometric_dispersion(const std::vector<verification::Point2D>& points) noexcept;

    /**
     * @brief Evaluates information gain for a candidate opportunity.
     */
    InformationGainResult evaluate(
        const graph::NodeId& kf_a,
        const graph::NodeId& kf_b,
        const graph::PoseGraph& graph,
        const std::vector<verification::Point2D>& candidate_landmarks) const;

private:
    InformationGainWeights m_weights;
};

} // namespace astromesh::coordination

