#include "armvision/kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace armvision {

namespace {

constexpr double kPi = std::numbers::pi;

constexpr double toRad(double deg) { return deg * kPi / 180.0; }
constexpr double toDeg(double rad) { return rad * 180.0 / kPi; }

/// Wraps an angle in radians to (-pi, pi].
double wrap(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0.0) a += 2.0 * kPi;
    return a - kPi;
}

}  // namespace

Eigen::Vector3d forwardKinematics(const JointAngles& q, const ArmConfig& cfg) {
    const double b = toRad(q.base);
    const double s = toRad(q.shoulder - 90.0);  // upper arm relative to horizontal
    const double e = toRad(q.elbow - 90.0);     // forearm relative to upper arm

    // Planar 2-link arm in the vertical plane selected by the base angle.
    const double r = cfg.upper_arm * std::cos(s) + cfg.forearm * std::cos(s + e);
    const double h = cfg.upper_arm * std::sin(s) + cfg.forearm * std::sin(s + e);

    return {r * std::cos(b), r * std::sin(b), cfg.shoulder_height + h};
}

std::optional<JointAngles> inverseKinematics(const Eigen::Vector3d& target, const ArmConfig& cfg,
                                             const JointAngles& reference) {
    const double l2 = cfg.upper_arm;
    const double l3 = cfg.forearm;
    const double h = target.z() - cfg.shoulder_height;
    const double rho = std::hypot(target.x(), target.y());

    // Base candidates: face the target (r = rho) or turn away and reach over the back (r = -rho).
    // Directly above the base the base angle is free, so keep the reference one.
    struct BaseCandidate {
        double base;  // rad
        double r;     // signed horizontal reach in the arm plane
    };
    std::array<BaseCandidate, 2> bases{};
    if (rho < 1e-9) {
        bases = {BaseCandidate{toRad(reference.base), 0.0}, BaseCandidate{toRad(reference.base), 0.0}};
    } else {
        const double b = std::atan2(target.y(), target.x());
        bases = {BaseCandidate{b, rho}, BaseCandidate{wrap(b + kPi), -rho}};
    }

    std::optional<JointAngles> best;
    double best_cost = std::numeric_limits<double>::infinity();

    for (const auto& [b, r] : bases) {
        // Law of cosines for the elbow.
        const double c = (r * r + h * h - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
        if (std::abs(c) > 1.0 + 1e-12) continue;  // out of reach
        const double e_abs = std::acos(std::clamp(c, -1.0, 1.0));

        for (const double e : {e_abs, -e_abs}) {
            const double s = std::atan2(h, r) - std::atan2(l3 * std::sin(e), l2 + l3 * std::cos(e));

            JointAngles q{toDeg(b), toDeg(wrap(s)) + 90.0, toDeg(e) + 90.0, reference.gripper};
            if (!withinLimits(q, cfg, 1e-6)) continue;
            q = clampToLimits(q, cfg);  // remove the tolerance overshoot

            const double cost = std::pow(q.base - reference.base, 2) +
                                std::pow(q.shoulder - reference.shoulder, 2) +
                                std::pow(q.elbow - reference.elbow, 2);
            if (cost < best_cost) {
                best_cost = cost;
                best = q;
            }
        }
    }
    return best;
}

}  // namespace armvision
