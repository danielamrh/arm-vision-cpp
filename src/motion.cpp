#include "armvision/motion.hpp"

// Motion planning: safety checks and paths made of small steps.
//
// EXERCISE: the parts marked TODO are left for you to implement, one per lesson
// (lesson 3 lives in include/armvision/motion.hpp, because templates must be in headers).
// Step-by-step guide: docs/tutorials/03-motion-planning.md
// Tests: tests/test_motion.cpp, no hardware needed:
//   Windows:  powershell -ExecutionPolicy Bypass -File tools/test.ps1 "Motion.*"
//   Linux:    ctest --preset linux -R Motion

#include <algorithm>
#include <cmath>

#include "armvision/kinematics.hpp"

namespace armvision {

std::string_view toString(Violation v) {
    // TODO 1a (lesson 1): a `switch` over the enum that returns a short English description
    //   for every value, e.g. "the gripper tip is too close to the table".
    (void)v;
    throw std::logic_error("toString(Violation): not implemented yet");
}

std::optional<Violation> findViolation(const JointAngles& q, const ArmConfig& cfg) {
    // TODO 1b (lesson 1): check the rules in the order of the enum and return the first one
    //   that is broken:
    //   - withinLimits(q, cfg, 1e-6)                      (arm_config.hpp)
    //   - forwardKinematics(q, cfg).z() >= cfg.min_height  (kinematics.hpp)
    //   - elbowPosition(q, cfg).z()     >= cfg.min_height
    //   Nothing broken: return std::nullopt.
    (void)q;
    (void)cfg;
    throw std::logic_error("findViolation: not implemented yet");
}

std::vector<JointAngles> interpolateJoints(const JointAngles& from, const JointAngles& to,
                                           double max_step_deg) {
    // TODO 2 (lesson 2):
    //   1. max_step_deg <= 0 -> throw std::invalid_argument
    //   2. largest = the largest |to.x - from.x| over the four joints (std::max with an
    //      initializer list takes any number of values: std::max({a, b, c, d}))
    //   3. largest == 0 -> return {} (nothing to do)
    //   4. steps = ceil(largest / max_step_deg); waypoint i (1..steps) is at fraction i / steps
    //      of the way from `from` to `to`, for all four joints
    (void)from;
    (void)to;
    (void)max_step_deg;
    throw std::logic_error("interpolateJoints: not implemented yet");
}

std::optional<std::vector<JointAngles>> straightLine(const JointAngles& from,
                                                     const Eigen::Vector3d& target,
                                                     const ArmConfig& cfg, double step_m) {
    // TODO 4 (lesson 4):
    //   1. step_m <= 0 -> throw std::invalid_argument
    //   2. start = forwardKinematics(from, cfg); steps = ceil(|target - start| / step_m), at
    //      least 1
    //   3. for i = 1..steps: point = start + (i / steps) * (target - start)
    //        q = inverseKinematics(point, cfg, reference)   with reference = previous waypoint
    //        (the first time: `from`)
    //        no solution or findViolation(*q, cfg) -> return std::nullopt
    //        otherwise append *q and use it as the next reference
    (void)from;
    (void)target;
    (void)cfg;
    (void)step_m;
    throw std::logic_error("straightLine: not implemented yet");
}

}  // namespace armvision
