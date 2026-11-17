#include "astromesh/geometry/se2.hpp"

#include <iostream>
#include <cmath>
#include <limits>
#include <vector>

using astromesh::geometry::SE2;
using astromesh::geometry::Twist2D;
using astromesh::verification::Point2D;
using astromesh::graph::Pose2D;
using astromesh::graph::normalize_angle;

static int g_total_assertions = 0;
static int g_failed_assertions = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        ++g_total_assertions; \
        if (!(cond)) { \
            ++g_failed_assertions; \
            std::cerr << "  FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

constexpr double PI = 3.14159265358979323846;
constexpr double EPS_LOOSE = 1e-6;
constexpr double EPS_TIGHT = 1e-9;

void test_identity_and_construction() {
    std::cout << "[TEST 1] Identity, Construction, and Pose2D Interoperability...\n";

    SE2 id = SE2::identity();
    TEST_ASSERT(id.is_valid(), "Identity must be valid");
    TEST_ASSERT(std::abs(id.x()) < EPS_TIGHT, "Identity x == 0");
    TEST_ASSERT(std::abs(id.y()) < EPS_TIGHT, "Identity y == 0");
    TEST_ASSERT(std::abs(id.theta()) < EPS_TIGHT, "Identity theta == 0");

    Pose2D p2d(1.5, -2.5, 0.75);
    SE2 se2_from_p(p2d);
    TEST_ASSERT(std::abs(se2_from_p.x() - 1.5) < EPS_TIGHT, "x matches Pose2D");
    TEST_ASSERT(std::abs(se2_from_p.y() - (-2.5)) < EPS_TIGHT, "y matches Pose2D");
    TEST_ASSERT(std::abs(se2_from_p.theta() - 0.75) < EPS_TIGHT, "theta matches Pose2D");

    Pose2D back = se2_from_p.to_pose2d();
    TEST_ASSERT(back == p2d, "to_pose2d() matches original Pose2D");
}

void test_composition_and_inverse() {
    std::cout << "[TEST 2] Composition, Inverses, and Group Associativity...\n";

    SE2 X1(1.0, 2.0, PI / 4.0);
    SE2 X2(-0.5, 1.5, -PI / 6.0);
    SE2 X3(2.0, -1.0, PI / 3.0);

    // Inverse axiom: X * X^{-1} == I, X^{-1} * X == I
    SE2 inv1 = X1.inverse();
    SE2 prod1 = X1 * inv1;
    SE2 prod2 = inv1 * X1;

    TEST_ASSERT(prod1 == SE2::identity(), "X1 * X1^{-1} == I");
    TEST_ASSERT(prod2 == SE2::identity(), "X1^{-1} * X1 == I");

    // Associativity: (X1 * X2) * X3 == X1 * (X2 * X3)
    SE2 assoc_left = (X1 * X2) * X3;
    SE2 assoc_right = X1 * (X2 * X3);
    TEST_ASSERT(assoc_left == assoc_right, "Group associativity (X1 * X2) * X3 == X1 * (X2 * X3)");

    // Hand-calculated composition:
    // X_a = (10, 5, pi/2), X_b = (1, 2, 0)
    // t_comp = R(pi/2) * [1, 2]^T + [10, 5]^T = [-2, 1]^T + [10, 5]^T = [8, 6]^T
    // theta_comp = pi/2
    SE2 X_a(10.0, 5.0, PI / 2.0);
    SE2 X_b(1.0, 2.0, 0.0);
    SE2 X_ab = X_a * X_b;
    TEST_ASSERT(std::abs(X_ab.x() - 8.0) < EPS_TIGHT, "Hand-calculated compose x == 8");
    TEST_ASSERT(std::abs(X_ab.y() - 6.0) < EPS_TIGHT, "Hand-calculated compose y == 6");
    TEST_ASSERT(std::abs(X_ab.theta() - PI / 2.0) < EPS_TIGHT, "Hand-calculated compose theta == pi/2");
}

void test_point_transformation() {
    std::cout << "[TEST 3] 2D Point Transformation and Round-Trip...\n";

    // Frame at (10, 5) rotated by 90 deg (pi/2)
    SE2 X(10.0, 5.0, PI / 2.0);
    Point2D p_body(1.0, 2.0);

    // p_world = R(pi/2) * [1, 2]^T + [10, 5]^T = [-2, 1]^T + [10, 5]^T = [8, 6]^T
    Point2D p_world = X.transform(p_body);
    TEST_ASSERT(std::abs(p_world.x - 8.0) < EPS_TIGHT, "Hand-calculated point transform x == 8");
    TEST_ASSERT(std::abs(p_world.y - 6.0) < EPS_TIGHT, "Hand-calculated point transform y == 6");

    // Round-trip: inverse_transform(p_world) == p_body
    Point2D p_recovered = X.inverse_transform(p_world);
    TEST_ASSERT(std::abs(p_recovered.x - 1.0) < EPS_TIGHT, "Inverse transform recovers body x");
    TEST_ASSERT(std::abs(p_recovered.y - 2.0) < EPS_TIGHT, "Inverse transform recovers body y");
}

void test_exp_and_log_maps() {
    std::cout << "[TEST 4] Lie Exponential and Logarithmic Maps...\n";

    // Identity mappings
    Twist2D zero_twist(0.0, 0.0, 0.0);
    SE2 exp_zero = SE2::exp(zero_twist);
    TEST_ASSERT(exp_zero == SE2::identity(), "exp([0, 0, 0]) == Identity");
    Twist2D log_zero = SE2::identity().log();
    TEST_ASSERT(log_zero == zero_twist, "log(Identity) == [0, 0, 0]");

    // Pure translation
    Twist2D trans_twist(3.5, -1.2, 0.0);
    SE2 exp_trans = SE2::exp(trans_twist);
    TEST_ASSERT(std::abs(exp_trans.x() - 3.5) < EPS_TIGHT, "Pure translation x");
    TEST_ASSERT(std::abs(exp_trans.y() - (-1.2)) < EPS_TIGHT, "Pure translation y");
    TEST_ASSERT(std::abs(exp_trans.theta()) < EPS_TIGHT, "Pure translation theta == 0");
    Twist2D log_trans = exp_trans.log();
    TEST_ASSERT(log_trans == trans_twist, "log(pure translation) recovers twist");

    // Pure rotation
    Twist2D rot_twist(0.0, 0.0, PI / 2.0);
    SE2 exp_rot = SE2::exp(rot_twist);
    TEST_ASSERT(std::abs(exp_rot.x()) < EPS_TIGHT, "Pure rotation x == 0");
    TEST_ASSERT(std::abs(exp_rot.y()) < EPS_TIGHT, "Pure rotation y == 0");
    TEST_ASSERT(std::abs(exp_rot.theta() - PI / 2.0) < EPS_TIGHT, "Pure rotation theta == pi/2");
    Twist2D log_rot = exp_rot.log();
    TEST_ASSERT(log_rot == rot_twist, "log(pure rotation) recovers twist");

    // Hand-calculated general twist:
    // xi = [1.0, 2.0, pi/3]^T
    // V(pi/3) * [1, 2]^T = [-0.1279363, 2.1314514]^T
    Twist2D general_twist(1.0, 2.0, PI / 3.0);
    SE2 exp_gen = SE2::exp(general_twist);
    TEST_ASSERT(std::abs(exp_gen.x() - (-0.1279363)) < EPS_LOOSE, "Hand-calculated exp x");
    TEST_ASSERT(std::abs(exp_gen.y() - 2.1314514) < EPS_LOOSE, "Hand-calculated exp y");
    TEST_ASSERT(std::abs(exp_gen.theta() - PI / 3.0) < EPS_TIGHT, "Hand-calculated exp theta");

    Twist2D log_gen = exp_gen.log();
    TEST_ASSERT(std::abs(log_gen.vx - 1.0) < EPS_LOOSE, "log recovers vx");
    TEST_ASSERT(std::abs(log_gen.vy - 2.0) < EPS_LOOSE, "log recovers vy");
    TEST_ASSERT(std::abs(log_gen.omega - PI / 3.0) < EPS_LOOSE, "log recovers omega");

    // Parameter sweep round-trip bijection: log(exp(xi)) == xi
    const std::vector<double> angles = {
        0.0, 1e-8, 1e-6, 1e-5, 0.01, 0.1, 0.5, 1.0, 2.0, PI / 2.0, PI - 0.01, -0.5, -PI / 2.0, -PI + 0.01
    };
    for (double th : angles) {
        Twist2D xi(1.23, -4.56, th);
        SE2 X = SE2::exp(xi);
        Twist2D recovered = X.log();
        TEST_ASSERT(std::abs(recovered.vx - xi.vx) < 1e-7, "Sweep round-trip vx");
        TEST_ASSERT(std::abs(recovered.vy - xi.vy) < 1e-7, "Sweep round-trip vy");
        TEST_ASSERT(std::abs(normalize_angle(recovered.omega - xi.omega)) < 1e-7, "Sweep round-trip omega");
    }
}

void test_small_angle_stability() {
    std::cout << "[TEST 5] Small-Angle Numerical Stability near Zero...\n";

    // Values crossing the 1e-6 Taylor expansion boundary
    const std::vector<double> tiny_omegas = {
        0.0, 1e-12, 1e-9, 5e-7, 9.9e-7, 1e-6, 1.01e-6, 5e-6, 1e-5
    };

    for (double w : tiny_omegas) {
        Twist2D xi(10.0, 20.0, w);
        SE2 X = SE2::exp(xi);

        TEST_ASSERT(std::isfinite(X.x()), "exp(xi).x must be finite");
        TEST_ASSERT(std::isfinite(X.y()), "exp(xi).y must be finite");
        TEST_ASSERT(std::isfinite(X.theta()), "exp(xi).theta must be finite");

        Twist2D rec = X.log();
        TEST_ASSERT(std::isfinite(rec.vx), "log(X).vx must be finite");
        TEST_ASSERT(std::isfinite(rec.vy), "log(X).vy must be finite");
        TEST_ASSERT(std::isfinite(rec.omega), "log(X).omega must be finite");

        TEST_ASSERT(std::abs(rec.vx - xi.vx) < 1e-8, "Small-angle round-trip vx precision");
        TEST_ASSERT(std::abs(rec.vy - xi.vy) < 1e-8, "Small-angle round-trip vy precision");
        TEST_ASSERT(std::abs(rec.omega - xi.omega) < 1e-8, "Small-angle round-trip omega precision");
    }
}

void test_angle_branch_cut_and_wrapping() {
    std::cout << "[TEST 6] Angle Wrapping and Branch Cut Behavior near +/- pi...\n";

    // Near positive boundary (+pi - eps)
    SE2 X_pos(1.0, 2.0, PI - 1e-5);
    TEST_ASSERT(std::abs(X_pos.theta() - (PI - 1e-5)) < EPS_TIGHT, "Theta near +pi preserved");

    // Near negative boundary (-pi + eps)
    SE2 X_neg(1.0, 2.0, -PI + 1e-5);
    TEST_ASSERT(std::abs(X_neg.theta() - (-PI + 1e-5)) < EPS_TIGHT, "Theta near -pi preserved");

    // Exact +pi wraps to -pi under [-pi, pi) convention
    SE2 X_pi(1.0, 2.0, PI, true);
    TEST_ASSERT(std::abs(X_pi.theta() - (-PI)) < EPS_TIGHT, "Theta at +pi wraps to -pi");

    // Multi-turn winding 3*pi wraps to -pi
    SE2 X_3pi(1.0, 2.0, 3.0 * PI, true);
    TEST_ASSERT(std::abs(X_3pi.theta() - (-PI)) < EPS_TIGHT, "Theta at 3*pi wraps to -pi");

    // Log map of pose near branch cut
    Twist2D log_pos = X_pos.log();
    TEST_ASSERT(std::abs(log_pos.omega - (PI - 1e-5)) < EPS_LOOSE, "log(X_pos) omega matches");
}

void test_adjoint_and_coadjoint() {
    std::cout << "[TEST 7] Adjoint Representation and Co-Adjoint Gyroscopic Invariance...\n";

    // 1. Adjoint Identity: Ad_I * xi == xi
    Twist2D xi(1.0, 2.0, 0.5);
    Twist2D adj_id = SE2::identity().adjoint(xi);
    TEST_ASSERT(adj_id == xi, "Ad_I * xi == xi");

    // 2. Adjoint composition: Ad_{X1 * X2} * xi == Ad_{X1} * (Ad_{X2} * xi)
    SE2 X1(2.0, -1.0, 0.4);
    SE2 X2(-0.5, 3.0, -0.8);
    Twist2D adj_comp = (X1 * X2).adjoint(xi);
    Twist2D adj_chain = X1.adjoint(X2.adjoint(xi));
    TEST_ASSERT(std::abs(adj_comp.vx - adj_chain.vx) < EPS_TIGHT, "Adjoint composition vx");
    TEST_ASSERT(std::abs(adj_comp.vy - adj_chain.vy) < EPS_TIGHT, "Adjoint composition vy");
    TEST_ASSERT(std::abs(adj_comp.omega - adj_chain.omega) < EPS_TIGHT, "Adjoint composition omega");

    // 3. Co-Adjoint Gyroscopic Invariance:
    // For ANY twist xi and momentum p, xi^T * ad_xi^*(p) == 0.
    // Test with both isotropic and anisotropic mass/inertia tensors!
    struct MassTensor { double mx; double my; double I; };
    std::vector<MassTensor> mass_tensors = {
        {1.0, 1.0, 1.0},   // Isotropic
        {2.5, 2.5, 0.8},   // Planar isotropic linear mass
        {1.0, 4.0, 2.0},   // Anisotropic linear mass
        {10.0, 0.5, 5.0}   // Extreme anisotropy
    };

    std::vector<Twist2D> test_twists = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 2.0, 0.0},
        {0.0, 0.0, 1.5},
        {1.2, -3.4, 2.5},
        {-4.0, 1.5, -1.8}
    };

    for (const auto& M : mass_tensors) {
        for (const auto& twist : test_twists) {
            Twist2D momentum(M.mx * twist.vx, M.my * twist.vy, M.I * twist.omega);
            Twist2D coadj = SE2::coadjoint(twist, momentum);

            // Power = xi^T * coadjoint
            double power = twist.vx * coadj.vx + twist.vy * coadj.vy + twist.omega * coadj.omega;
            TEST_ASSERT(std::abs(power) < 1e-12, "Gyroscopic power xi^T * ad_xi^*(M*xi) == 0 identically");
        }
    }
}

void test_nonfinite_and_invalid_inputs() {
    std::cout << "[TEST 8] Non-Finite and Invalid Input Rejection...\n";

    double nan_val = std::numeric_limits<double>::quiet_NaN();
    double inf_val = std::numeric_limits<double>::infinity();

    SE2 nan_pose(nan_val, 0.0, 0.0);
    TEST_ASSERT(!nan_pose.is_valid(), "NaN x must be invalid");

    SE2 inf_pose(0.0, inf_val, 0.0);
    TEST_ASSERT(!inf_pose.is_valid(), "Inf y must be invalid");

    Twist2D nan_twist(0.0, 0.0, nan_val);
    TEST_ASSERT(!nan_twist.is_valid(), "NaN twist must be invalid");

    // Operations with invalid inputs must produce invalid outputs without crashing
    SE2 comp_res = nan_pose * SE2::identity();
    TEST_ASSERT(!comp_res.is_valid(), "Compose with NaN pose produces invalid SE2");

    SE2 inv_res = inf_pose.inverse();
    TEST_ASSERT(!inv_res.is_valid(), "Inverse of Inf pose produces invalid SE2");

    Point2D pt_res = nan_pose.transform(Point2D(1.0, 1.0));
    TEST_ASSERT(!pt_res.is_valid(), "Transform with NaN pose produces invalid Point2D");

    SE2 exp_res = SE2::exp(nan_twist);
    TEST_ASSERT(!exp_res.is_valid(), "exp(NaN twist) produces invalid SE2");

    Twist2D log_res = nan_pose.log();
    TEST_ASSERT(!log_res.is_valid(), "log(NaN pose) produces invalid Twist2D");

    Twist2D adj_res = nan_pose.adjoint(Twist2D(1.0, 1.0, 1.0));
    TEST_ASSERT(!adj_res.is_valid(), "Adjoint of NaN pose produces invalid Twist2D");

    Twist2D coadj_res = SE2::coadjoint(nan_twist, Twist2D(1.0, 1.0, 1.0));
    TEST_ASSERT(!coadj_res.is_valid(), "Coadjoint with NaN twist produces invalid Twist2D");
}

void test_coadjoint_duality_pairing() {
    std::cout << "[TEST 9] Co-Adjoint Duality Pairing: <ad_xi^* p, eta> == <p, ad_xi eta>...\n";

    // Lie algebra bracket adjoint: ad_xi * eta = [ -w * eta_y + vy * eta_w; w * eta_x - vx * eta_w; 0 ]
    auto eval_ad = [](const Twist2D& xi, const Twist2D& eta) -> Twist2D {
        return Twist2D(
            -xi.omega * eta.vy + xi.vy * eta.omega,
             xi.omega * eta.vx - xi.vx * eta.omega,
             0.0
        );
    };

    std::vector<Twist2D> twists = {
        {1.5, -2.0, 0.8},
        {-3.0, 1.2, -1.5},
        {0.5, 4.0, 2.2}
    };
    std::vector<Twist2D> momenta = {
        {2.0, -1.0, 0.5},
        {-1.5, 3.5, 2.0},
        {4.0, 2.0, -3.0}
    };
    std::vector<Twist2D> etas = {
        {0.3, 0.7, -0.4},
        {-1.0, 2.0, 1.5},
        {2.5, -1.5, 0.9}
    };

    for (const auto& xi : twists) {
        for (const auto& p : momenta) {
            for (const auto& eta : etas) {
                // 1. Dual pairing: < ad_xi^*(p), eta >
                Twist2D coadj = SE2::coadjoint(xi, p);
                double lhs = coadj.vx * eta.vx + coadj.vy * eta.vy + coadj.omega * eta.omega;

                // 2. Action pairing: < p, ad_xi(eta) >
                Twist2D ad_eta = eval_ad(xi, eta);
                double rhs = p.vx * ad_eta.vx + p.vy * ad_eta.vy + p.omega * ad_eta.omega;

                TEST_ASSERT(std::abs(lhs - rhs) < 1e-12, "Duality pairing <ad_xi^* p, eta> == <p, ad_xi eta>");

                // 3. Contrast with flawed third-component sign [omega py, -omega px, vx py - vy px]
                Twist2D coadj_flawed(
                     xi.omega * p.vy,
                    -xi.omega * p.vx,
                     xi.vx * p.vy - xi.vy * p.vx
                );
                double lhs_flawed = coadj_flawed.vx * eta.vx + coadj_flawed.vy * eta.vy + coadj_flawed.omega * eta.omega;
                const double diff_flawed = std::abs(lhs_flawed - rhs);
                if (std::abs(eta.omega) > 1e-3 && std::abs(xi.vy * p.vx - xi.vx * p.vy) > 1e-3) {
                    TEST_ASSERT(diff_flawed > 1e-6, "Flawed coadjoint expression strictly fails duality pairing");
                }
            }
        }
    }
}

void test_coadjoint_counterexample() {
    std::cout << "[TEST 10] Co-Adjoint Power Counterexample under Flawed Expression...\n";

    // Nondegenerate state with anisotropic mass
    const double mx = 1.0, my = 4.0, Itheta = 2.0;
    const Twist2D xi(1.2, -3.4, 2.5);
    const Twist2D momentum(mx * xi.vx, my * xi.vy, Itheta * xi.omega); // [1.2, -13.6, 5.0]

    // Implemented coadjoint: [omega py, -omega px, vy px - vx py]
    Twist2D coadj_correct = SE2::coadjoint(xi, momentum);
    double power_correct = xi.vx * coadj_correct.vx + xi.vy * coadj_correct.vy + xi.omega * coadj_correct.omega;
    TEST_ASSERT(std::abs(power_correct) < 1e-12, "Implemented coadjoint strictly satisfies xi^T * ad_xi^*(M*xi) == 0");

    // Flawed reported expression: [omega py, -omega px, vx py - vy px]
    Twist2D coadj_flawed(
         xi.omega * momentum.vy,
        -xi.omega * momentum.vx,
         xi.vx * momentum.vy - xi.vy * momentum.vx
    );
    double power_flawed = xi.vx * coadj_flawed.vx + xi.vy * coadj_flawed.vy + xi.omega * coadj_flawed.omega;

    // Algebraic value: 2 * omega * (vx * py - vy * px) = 2 * 2.5 * (1.2 * (-13.6) - (-3.4) * 1.2) = 5.0 * (-12.24) = -61.2
    TEST_ASSERT(std::abs(power_flawed - (-61.2)) < 1e-9, "Flawed expression yields exactly -61.2 W power (non-zero counterexample)");
    TEST_ASSERT(std::abs(power_flawed) > 1.0, "Flawed expression strictly violates gyroscopic invariance");
}

int main() {
    std::cout << "=================================================\n";
    std::cout << " AstroMesh-X: SE(2) Lie-Group Operations Suite\n";
    std::cout << "=================================================\n";

    test_identity_and_construction();
    test_composition_and_inverse();
    test_point_transformation();
    test_exp_and_log_maps();
    test_small_angle_stability();
    test_angle_branch_cut_and_wrapping();
    test_adjoint_and_coadjoint();
    test_nonfinite_and_invalid_inputs();
    test_coadjoint_duality_pairing();
    test_coadjoint_counterexample();

    std::cout << "=================================================\n";
    std::cout << " Total assertions: " << g_total_assertions << "\n";
    std::cout << " Failed assertions: " << g_failed_assertions << "\n";
    std::cout << "=================================================\n";

    return (g_failed_assertions == 0) ? 0 : 1;
}
