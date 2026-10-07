#pragma once

#include <optional>

#include <Eigen/Core>

#include "armvision/arm_config.hpp"

namespace armvision {

// Base frame: origin at the centre of the base on the table, z up,
// x along base angle 0°, y along base angle 90°. Units: metres.
//
// Servo convention (same as robot_arm_env.py):
//   shoulder 90° -> upper arm horizontal, larger values lift it
//   elbow    90° -> forearm in line with the upper arm

/// Position of the gripper tip in the base frame.
[[nodiscard]] Eigen::Vector3d forwardKinematics(const JointAngles& q, const ArmConfig& cfg);

/// Closed-form inverse kinematics for base, shoulder and elbow.
/// Up to four solutions exist (reaching over the front or the back of the base, elbow up or
/// down). The one inside the joint limits that is closest to `reference` is returned.
/// The gripper angle is taken from `reference`. std::nullopt if the target is unreachable.
[[nodiscard]] std::optional<JointAngles> inverseKinematics(const Eigen::Vector3d& target,
                                                           const ArmConfig& cfg,
                                                           const JointAngles& reference = {});

}  // namespace armvision
