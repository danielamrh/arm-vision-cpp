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
inline double smoothstep(double t) {
    // TODO 3a (lesson 3): the classic smoothstep polynomial 3t^2 - 2t^3.
    (void)t;
    throw std::logic_error("smoothstep: not implemented yet");
}

/// `steps` waypoints from `from` (exclusive) to `to` (inclusive). Waypoint i (1..steps) is at
/// progress ease(i / steps) between the two poses, for all four joints.
template <Easing E>
[[nodiscard]] std::vector<JointAngles> interpolateEased(const JointAngles& from,
                                                        const JointAngles& to, std::size_t steps,
                                                        E ease) {
    // TODO 3b (lesson 3): for i = 1..steps: t = i / steps, a = ease(t),
    //   joint = from + a * (to - from) for base, shoulder, elbow and gripper.
    (void)from;
    (void)to;
    (void)steps;
    (void)ease;
    throw std::logic_error("interpolateEased: not implemented yet");
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
