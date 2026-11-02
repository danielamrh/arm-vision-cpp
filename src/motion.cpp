#include "armvision/motion.hpp"

// Motion planning: safety checks and paths made of small steps.
//
// Written step by step in docs/tutorials/03-motion-planning.md (lesson 3, the templates, lives
// in include/armvision/motion.hpp). Tests: tests/test_motion.cpp, no hardware needed:
//   Windows:  powershell -ExecutionPolicy Bypass -File tools/test.ps1 "Motion.*"
//   Linux:    ctest --preset linux -R Motion

#include <algorithm>
#include <cmath>
#include <string>

#include "armvision/kinematics.hpp"

namespace armvision {

namespace {

/// Number of equal steps needed to cover `distance` with steps of at most `max_step`, at least 1.
/// Throws std::invalid_argument for a step that is not positive (including NaN) and for absurd
/// requests: converting a huge or NaN double to an integer would be undefined behaviour.
std::size_t stepCount(double distance, double max_step, const char* caller) {
    if (!(max_step > 0.0)) {
        throw std::invalid_argument(std::string(caller) + ": step must be positive");
    }
    constexpr double kMaxSteps = 1e6;
    const double steps = std::ceil(distance / max_step);
    if (!(steps <= kMaxSteps)) {  // also true for NaN
        throw std::invalid_argument(std::string(caller) + ": too many steps (or invalid input)");
    }
    return std::max<std::size_t>(1, static_cast<std::size_t>(steps));
}

}  // namespace

std::string_view toString(Violation v) {
    // Lower case: the text is used inside sentences ("refusing to move: ...").
    switch (v) {
        case Violation::JointLimit: return "a joint is outside its limits";
        case Violation::TipBelowTable: return "the gripper tip is below the minimum height";
        case Violation::ElbowBelowTable: return "the elbow is below the minimum height";
    }
    return "unknown violation";
}

std::optional<Violation> findViolation(const JointAngles& q, const ArmConfig& cfg) {
    // Small tolerance: poses computed by IK land on a limit like 150.0000000001.
    if (!withinLimits(q, cfg, 1e-6)) return Violation::JointLimit;
    if (forwardKinematics(q, cfg).z() < cfg.min_height) return Violation::TipBelowTable;
    if (elbowPosition(q, cfg).z() < cfg.min_height) return Violation::ElbowBelowTable;
    return std::nullopt;
}

std::vector<JointAngles> interpolateJoints(const JointAngles& from, const JointAngles& to,
                                           double max_step_deg) {
    // The joint with the largest move sets the number of steps, so all joints arrive together.
    const double largest = std::max({std::abs(to.base - from.base),
                                     std::abs(to.shoulder - from.shoulder),
                                     std::abs(to.elbow - from.elbow),
                                     std::abs(to.gripper - from.gripper)});
    const std::size_t steps = stepCount(largest, max_step_deg, "interpolateJoints");
    if (largest == 0.0) return {};
    return interpolateEased(from, to, steps, linear);  // evenly spaced = linear easing
}

std::optional<std::vector<JointAngles>> straightLine(const JointAngles& from,
                                                     const Eigen::Vector3d& target,
                                                     const ArmConfig& cfg, double step_m) {
    const Eigen::Vector3d start = forwardKinematics(from, cfg);
    const std::size_t steps = stepCount((target - start).norm(), step_m, "straightLine");

    std::vector<JointAngles> waypoints;
    waypoints.reserve(steps);
    JointAngles reference = from;  // keeps IK on the same solution branch from step to step
    for (std::size_t i = 1; i <= steps; ++i) {
        const double fraction = static_cast<double>(i) / static_cast<double>(steps);
        const Eigen::Vector3d point = start + fraction * (target - start);
        const auto q = inverseKinematics(point, cfg, reference);
        if (!q || findViolation(*q, cfg)) return std::nullopt;  // unreachable or unsafe
        waypoints.push_back(*q);
        reference = *q;
    }
    return waypoints;
}

}  // namespace armvision
