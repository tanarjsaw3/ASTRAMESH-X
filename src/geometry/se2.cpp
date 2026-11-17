#include "astromesh/geometry/se2.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

namespace astromesh::geometry {

std::string Twist2D::to_string() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "Twist2D(vx=" << vx << ", vy=" << vy << ", omega=" << omega << ")";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Twist2D& twist) {
    return os << twist.to_string();
}

SE2::SE2(double x, double y, double theta, bool normalize) noexcept
    : x_(x), y_(y), theta_(normalize ? graph::normalize_angle(theta) : theta) {}

SE2::SE2(const graph::Pose2D& pose) noexcept
    : x_(pose.x), y_(pose.y), theta_(pose.theta) {}

SE2 SE2::compose(const SE2& other) const noexcept {
    if (!is_valid() || !other.is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return SE2(NAN_VAL, NAN_VAL, NAN_VAL, false);
    }

    const double c1 = cos_theta();
    const double s1 = sin_theta();

    const double comp_x = x_ + c1 * other.x_ - s1 * other.y_;
    const double comp_y = y_ + s1 * other.x_ + c1 * other.y_;
    const double comp_theta = graph::normalize_angle(theta_ + other.theta_);

    return SE2(comp_x, comp_y, comp_theta, false);
}

SE2 SE2::inverse() const noexcept {
    if (!is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return SE2(NAN_VAL, NAN_VAL, NAN_VAL, false);
    }

    const double c = cos_theta();
    const double s = sin_theta();

    const double inv_x = -c * x_ - s * y_;
    const double inv_y =  s * x_ - c * y_;
    const double inv_theta = graph::normalize_angle(-theta_);

    return SE2(inv_x, inv_y, inv_theta, false);
}

verification::Point2D SE2::transform(const verification::Point2D& p_body) const noexcept {
    if (!is_valid() || !p_body.is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return verification::Point2D(NAN_VAL, NAN_VAL);
    }

    const double c = cos_theta();
    const double s = sin_theta();

    const double px_parent = x_ + c * p_body.x - s * p_body.y;
    const double py_parent = y_ + s * p_body.x + c * p_body.y;

    return verification::Point2D(px_parent, py_parent);
}

verification::Point2D SE2::inverse_transform(const verification::Point2D& p_parent) const noexcept {
    if (!is_valid() || !p_parent.is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return verification::Point2D(NAN_VAL, NAN_VAL);
    }

    const double dx = p_parent.x - x_;
    const double dy = p_parent.y - y_;

    const double c = cos_theta();
    const double s = sin_theta();

    const double px_body =  c * dx + s * dy;
    const double py_body = -s * dx + c * dy;

    return verification::Point2D(px_body, py_body);
}

SE2 SE2::exp(const Twist2D& twist) noexcept {
    if (!twist.is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return SE2(NAN_VAL, NAN_VAL, NAN_VAL, false);
    }

    const double w = twist.omega;
    double a = 0.0;
    double b = 0.0;

    // Small-angle threshold: |w| < 1e-6
    if (std::abs(w) < 1e-6) {
        const double w2 = w * w;
        // sin(w)/w = 1 - w^2/6 + w^4/120
        a = 1.0 - w2 / 6.0 + (w2 * w2) / 120.0;
        // (1 - cos(w))/w = w/2 - w^3/24 + w^5/720
        b = 0.5 * w - (w * w2) / 24.0 + (w * w2 * w2) / 720.0;
    } else {
        a = std::sin(w) / w;
        b = (1.0 - std::cos(w)) / w;
    }

    const double exp_x = a * twist.vx - b * twist.vy;
    const double exp_y = b * twist.vx + a * twist.vy;
    const double exp_theta = graph::normalize_angle(w);

    return SE2(exp_x, exp_y, exp_theta, false);
}

Twist2D SE2::log() const noexcept {
    if (!is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return Twist2D(NAN_VAL, NAN_VAL, NAN_VAL);
    }

    const double th = graph::normalize_angle(theta_);
    double alpha = 0.0;
    const double beta = 0.5 * th;

    // Small-angle threshold: |th| < 1e-6
    if (std::abs(th) < 1e-6) {
        const double th2 = th * th;
        // (th/2)/tan(th/2) = 1 - th^2/12 - th^4/720
        alpha = 1.0 - th2 / 12.0 - (th2 * th2) / 720.0;
    } else {
        const double half_th = 0.5 * th;
        alpha = half_th / std::tan(half_th);
    }

    const double vx =  alpha * x_ + beta * y_;
    const double vy = -beta * x_ + alpha * y_;

    return Twist2D(vx, vy, th);
}

Twist2D SE2::adjoint(const Twist2D& twist) const noexcept {
    if (!is_valid() || !twist.is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return Twist2D(NAN_VAL, NAN_VAL, NAN_VAL);
    }

    const double c = cos_theta();
    const double s = sin_theta();

    const double adj_vx = c * twist.vx - s * twist.vy + y_ * twist.omega;
    const double adj_vy = s * twist.vx + c * twist.vy - x_ * twist.omega;
    const double adj_w  = twist.omega;

    return Twist2D(adj_vx, adj_vy, adj_w);
}

Twist2D SE2::coadjoint(const Twist2D& xi, const Twist2D& momentum) noexcept {
    if (!xi.is_valid() || !momentum.is_valid()) {
        constexpr double NAN_VAL = std::numeric_limits<double>::quiet_NaN();
        return Twist2D(NAN_VAL, NAN_VAL, NAN_VAL);
    }

    // Dual pairing under standard Euclidean coordinates on se(2):
    // ad_xi^* p = [   omega * py        ]
    //             [  -omega * px        ]
    //             [  vy * px - vx * py  ]
    const double co_vx =  xi.omega * momentum.vy;
    const double co_vy = -xi.omega * momentum.vx;
    const double co_w  =  xi.vy * momentum.vx - xi.vx * momentum.vy;

    return Twist2D(co_vx, co_vy, co_w);
}

std::string SE2::to_string() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "SE2(x=" << x_ << ", y=" << y_ << ", theta=" << theta_ << ")";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const SE2& pose) {
    return os << pose.to_string();
}

} // namespace astromesh::geometry
