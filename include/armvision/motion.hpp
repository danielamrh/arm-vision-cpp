#pragma once

// Motion planning for the arm: safety checks and paths made of small steps.
// Everything here is pure computation (no hardware), see docs/tutorials/03-motion-planning.md.

#include <concepts>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <vector>

#include <Eigen/Core>

#include "armvision/arm_config.hpp"

namespace armvision {

// ── Lesson 1: safety ──────────────────────────────────────────

/// Rules a pose can break, checked in this order.
enum class Violation {
    JointLimit,       // a joint outside its limits
    TipBelowTable,    // gripper tip lower than cfg.min_height
    ElbowBelowTable,  // elbow joint lower than cfg.min_height
};

[[nodiscard]] std::string_view toString(Violation v);

/// The first rule `q` breaks, or std::nullopt if the pose is safe.
[[nodiscard]] std::optional<Violation> findViolation(const JointAngles& q, const ArmConfig& cfg);

// ── Lesson 2: joint-space interpolation ───────────────────────

/// Waypoints from `from` (exclusive) to `to` (inclusive), evenly spaced so that no joint moves
/// more than `max_step_deg` between two consecutive waypoints. Empty if from == to.
/// Throws std::invalid_argument if max_step_deg <= 0.
[[nodiscard]] std::vector<JointAngles> interpolateJoints(const JointAngles& from,
                                                         const JointAngles& to,
                                                         double max_step_deg);

// ── Lesson 3: easing (templates and concepts) ─────────────────

/// Anything callable as f(double) -> double: maps progress t in [0, 1] to [0, 1].
template <typename F>
concept Easing = std::invocable<F, double> &&
                 std::convertible_to<std::invoke_result_t<F, double>, double>;

/// Constant speed.
inline double linear(double t) { return t; }

/// Starts and stops smoothly: f(0) = 0, f(1) = 1, and zero slope at both ends.
inline double smoothstep(double t) { return t * t * (3.0 - 2.0 * t); }

/// The pose a fraction `a` of the way from `from` to `to` (a = 0: from, a = 1: to).
[[nodiscard]] inline JointAngles lerp(const JointAngles& from, const JointAngles& to, double a) {
    return {from.base + a * (to.base - from.base),
            from.shoulder + a * (to.shoulder - from.shoulder),
            from.elbow + a * (to.elbow - from.elbow),
            from.gripper + a * (to.gripper - from.gripper)};
}

/// `steps` waypoints from `from` (exclusive) to `to` (inclusive). Waypoint i (1..steps) is at
/// progress ease(i / steps) between the two poses, for all four joints.
template <Easing E>
[[nodiscard]] std::vector<JointAngles> interpolateEased(const JointAngles& from,
                                                        const JointAngles& to, std::size_t steps,
                                                        E ease) {
    std::vector<JointAngles> waypoints;
    waypoints.reserve(steps);
    for (std::size_t i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        waypoints.push_back(lerp(from, to, ease(t)));
    }
    return waypoints;
}

// ── Lesson 4: straight lines in space ─────────────────────────

/// Joint waypoints that move the tip along a straight line from where `from` puts it to
/// `target`, in steps of at most `step_m` metres (`from` itself is not included, the last
/// waypoint reaches `target`). Every waypoint is solved with inverse kinematics, using the
/// previous one as reference so the arm does not flip between solutions.
/// std::nullopt if any point on the line is unreachable or unsafe.
[[nodiscard]] std::optional<std::vector<JointAngles>> straightLine(const JointAngles& from,
                                                                   const Eigen::Vector3d& target,
                                                                   const ArmConfig& cfg,
                                                                   double step_m);

}  // namespace armvision
