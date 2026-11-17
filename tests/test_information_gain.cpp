#include "astromesh/coordination/information_gain.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace astromesh;
using namespace astromesh::graph;
using namespace astromesh::verification;
using namespace astromesh::coordination;

static bool approx_equal(double a, double b, double eps = 1e-4) {
    return std::abs(a - b) <= eps;
}

void test_geometric_dispersion_calculation() {
    std::cout << "[RUN] test_geometric_dispersion_calculation..." << std::endl;

    // Fewer than 3 points -> 0.0
    std::vector<Point2D> pts_few = {{0, 0}, {1, 1}};
    assert(InformationGainEvaluator::compute_geometric_dispersion(pts_few) == 0.0);

    // Collinear points along X axis -> det should be 0.0
    std::vector<Point2D> pts_collinear = {{0, 0}, {2, 0}, {4, 0}, {6, 0}};
    double det_collinear = InformationGainEvaluator::compute_geometric_dispersion(pts_collinear);
    assert(approx_equal(det_collinear, 0.0, 1e-5));

    // Well-dispersed 2D square layout: (0,0), (4,0), (4,4), (0,4)
    std::vector<Point2D> pts_square = {{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    double det_square = InformationGainEvaluator::compute_geometric_dispersion(pts_square);
    assert(det_square > 1.0); // Var_x = 4, Var_y = 4, Cov = 0 -> det = 16

    std::cout << "[PASS] test_geometric_dispersion_calculation" << std::endl;
}

void test_information_gain_evaluation() {
    std::cout << "[RUN] test_information_gain_evaluation..." << std::endl;

    InformationGainEvaluator evaluator;
    PoseGraph graph;

    // Disconnected rovers: Rover 1 and Rover 2 have no verified edges
    graph.add_node(KeyframeNode(NodeId(1, 5), Pose2D(0.0, 0.0, 0.0)));
    graph.add_node(KeyframeNode(NodeId(2, 5), Pose2D(10.0, 0.0, 0.0)));

    std::vector<Point2D> dispersed_lms = {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}};
    auto res_unbridged = evaluator.evaluate(NodeId(1, 5), NodeId(2, 5), graph, dispersed_lms);
    assert(!res_unbridged.is_degenerate);
    assert(res_unbridged.topological_component == 1.0); // 1.0 / (1 + 0)
    assert(res_unbridged.total_score > 3.0);

    // Collinear points -> rejected as degenerate
    std::vector<Point2D> collinear_lms = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    auto res_collinear = evaluator.evaluate(NodeId(1, 5), NodeId(2, 5), graph, collinear_lms);
    assert(res_collinear.is_degenerate);
    assert(res_collinear.total_score == 0.0);

    // Add existing verified edges between Rover 1 and Rover 2
    for (uint32_t i = 1; i <= 5; ++i) {
        graph.add_node(KeyframeNode(NodeId(1, i), Pose2D(static_cast<double>(i), 0, 0)));
        graph.add_node(KeyframeNode(NodeId(2, i), Pose2D(static_cast<double>(i), 0, 0)));
        PoseGraphEdge edge(i, NodeId(1, i), NodeId(2, i),
                           EdgeType::InterRoverLoopClosure, Pose2D(1, 0, 0),
                           InformationMatrix2D::diagonal(100, 1000),
                           EdgeVerificationStatus::Verified);
        graph.add_edge(edge);
    }

    // Now rovers are saturated with 5 edges
    auto res_saturated = evaluator.evaluate(NodeId(1, 10), NodeId(2, 10), graph, dispersed_lms);
    assert(res_saturated.topological_component < 0.20); // 1.0 / (1 + 5) = 0.166
    assert(res_saturated.redundancy_penalty > 0.0);
    assert(res_saturated.total_score < res_unbridged.total_score);

    // Exact direct duplicate edge
    auto res_dup = evaluator.evaluate(NodeId(1, 1), NodeId(2, 1), graph, dispersed_lms);
    assert(res_dup.total_score == 0.0);
    assert(res_dup.redundancy_penalty >= 10.0);

    std::cout << "[PASS] test_information_gain_evaluation" << std::endl;
}

int main() {
    std::cout << "=============================================" << std::endl;
    std::cout << " AstroMesh-X: Information Gain Evaluator Tests" << std::endl;
    std::cout << "=============================================" << std::endl;

    test_geometric_dispersion_calculation();
    test_information_gain_evaluation();

    std::cout << "All Information Gain tests PASSED." << std::endl;
    return 0;
}
