#pragma once

#include "astromesh/graph/pose_graph.hpp"
#include "astromesh/verification/geometric_verifier.hpp"

#include <cmath>
#include <string>
#include <iosfwd>
#include <limits>

namespace astromesh::geometry {

/**
 * @brief Represents a tangent vector (twist) in the Lie algebra se(2) = T_I SE(2).
 *
 * Vector Coordinate Convention:
 *   xi = [vx, vy, omega]^T
 *   - vx: Linear velocity or displacement along body x-axis (meters or m/s).
 *   - vy: Linear velocity or displacement along body y-axis (meters or m/s).
 *   - omega: Heading angular velocity or rotation angle (radians or rad/s).
 *
 * Matrix Generator (wedge operator):
 *   xi^wedge = [  0     -omega   vx ]
 *              [  omega  0       vy ]
 *              [  0      0       0  ]
 */
struct Twist2D {
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};

    constexpr Twist2D() noexcept = default;
    constexpr Twist2D(double p_vx, double p_vy, double p_omega) noexcept
        : vx(p_vx), vy(p_vy), omega(p_omega) {}

    bool is_valid() const noexcept {
        return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(omega);
    }

    bool operator==(const Twist2D& other) const noexcept {
        constexpr double EPS = 1e-9;
        return std::abs(vx - other.vx) < EPS &&
               std::abs(vy - other.vy) < EPS &&
               std::abs(omega - other.omega) < EPS;
    }

    bool operator!=(const Twist2D& other) const noexcept {
        return !(*this == other);
    }

    std::string to_string() const;
};

std::ostream& operator<<(std::ostream& os, const Twist2D& twist);

/**
 * @brief Represents an element of the Special Euclidean Group SE(2) = SO(2) |x| R^2.
 *
 * Coordinate Convention:
 *   Rigid transformation matrix:
 *     X = [ cos(theta)  -sin(theta)  x ]
 *         [ sin(theta)   cos(theta)  y ]
 *         [ 0            0           1 ]
 *
 *   - x: Translation in meters along x-axis of reference frame.
 *   - y: Translation in meters along y-axis of reference frame.
 *   - theta: Planar orientation / yaw angle in radians, normalized to [-pi, pi).
 *
 * Point Transformation:
 *   p_world = X * p_body = R(theta) * p_body + t
 *   [ px_world ]   [ cos(theta) -sin(theta) ] [ px_body ]   [ x ]
 *   [ py_world ] = [ sin(theta)  cos(theta) ] [ py_body ] + [ y ]
 */
class SE2 {
public:
    constexpr SE2() noexcept : x_(0.0), y_(0.0), theta_(0.0) {}

    SE2(double x, double y, double theta, bool normalize = true) noexcept;

    explicit SE2(const graph::Pose2D& pose) noexcept;

    static SE2 identity() noexcept {
        return SE2(0.0, 0.0, 0.0);
    }

    static SE2 from_xy_theta(double x, double y, double theta) noexcept {
        return SE2(x, y, theta);
    }

    double x() const noexcept { return x_; }
    double y() const noexcept { return y_; }
    double theta() const noexcept { return theta_; }

    double cos_theta() const noexcept { return std::cos(theta_); }
    double sin_theta() const noexcept { return std::sin(theta_); }

    graph::Pose2D to_pose2d() const noexcept {
        return graph::Pose2D(x_, y_, theta_, false);
    }

    bool is_valid() const noexcept {
        return std::isfinite(x_) && std::isfinite(y_) && std::isfinite(theta_);
    }

    bool operator==(const SE2& other) const noexcept {
        constexpr double EPS = 1e-9;
        return std::abs(x_ - other.x_) < EPS &&
               std::abs(y_ - other.y_) < EPS &&
               std::abs(graph::normalize_angle(theta_ - other.theta_)) < EPS;
    }

    bool operator!=(const SE2& other) const noexcept {
        return !(*this == other);
    }

    /**
     * @brief Group composition: X_comp = (*this) * other.
     *        R_comp = R1 * R2
     *        t_comp = R1 * t2 + t1
     */
    SE2 compose(const SE2& other) const noexcept;
    SE2 operator*(const SE2& other) const noexcept {
        return compose(other);
    }

    /**
     * @brief Group inverse: X^{-1}.
     *        R_{inv} = R^T
     *        t_{inv} = -R^T * t
     *        theta_{inv} = -theta
     */
    SE2 inverse() const noexcept;

    /**
     * @brief Transforms a 2D point from local body frame to parent frame:
     *        p_parent = R * p_body + t.
     */
    verification::Point2D transform(const verification::Point2D& p_body) const noexcept;

    /**
     * @brief Transforms a 2D point from parent frame into local body frame:
     *        p_body = R^T * (p_parent - t).
     */
    verification::Point2D inverse_transform(const verification::Point2D& p_parent) const noexcept;

    /**
     * @brief Exponential map from Lie algebra se(2) to Lie group SE(2).
     *        Uses numerically stable Taylor expansions for |omega| < 1e-6.
     */
    static SE2 exp(const Twist2D& twist) noexcept;

    /**
     * @brief Logarithmic map from Lie group SE(2) to Lie algebra se(2).
     *        Uses numerically stable Taylor expansions for |theta| < 1e-6.
     */
    Twist2D log() const noexcept;

    /**
     * @brief Adjoint representation Ad_X: se(2) -> se(2).
     *        For SE(2):
     *        Ad_X * xi = [ R * v + [y; -x] * omega ]
     *                    [         omega           ]
     */
    Twist2D adjoint(const Twist2D& twist) const noexcept;

    /**
     * @brief Co-adjoint operator ad_xi^*: se(2)^* -> se(2)^* for momentum p = [px, py, ptheta]^T.
     *        Under the matrix Lie algebra bracket commutator [xi1, xi2] on se(2):
     *          ad_xi * eta = [ -omega * vy_eta + omega_eta * vy ]
     *                        [  omega * vx_eta - omega_eta * vx ]
     *                        [                 0                ]
     *        The dual co-adjoint operator under standard Euclidean pairing is:
     *          ad_xi^* p = [   omega * py        ]
     *                      [  -omega * px        ]
     *                      [  vy * px - vx * py  ]
     *
     *        Invariant: For all xi and all momentum p:
     *          xi^T * (ad_xi^* p) == 0 (zero gyroscopic power).
     */
    static Twist2D coadjoint(const Twist2D& xi, const Twist2D& momentum) noexcept;

    std::string to_string() const;

private:
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
};

std::ostream& operator<<(std::ostream& os, const SE2& pose);

} // namespace astromesh::geometry
