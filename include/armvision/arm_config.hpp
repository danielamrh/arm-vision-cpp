#pragma once

namespace armvision {

/// Joint angles in servo degrees (0..180), exactly what the firmware expects.
struct JointAngles {
    double base{90.0};
    double shoulder{90.0};
    double elbow{90.0};
    double gripper{90.0};

    friend bool operator==(const JointAngles&, const JointAngles&) = default;
};

struct JointLimits {
    double min;
    double max;
};

/// Geometry and joint limits of the Spacnana SNAM1500 arm.
/// Defaults match rl_robot_arm/robot_arm_env.py and firmware/src/main.cpp.
struct ArmConfig {
    // Link lengths in metres (measured from photos -> calibrate them with the camera later).
    double shoulder_height{0.08};  // L1: table -> shoulder axis
    double upper_arm{0.10};        // L2: shoulder -> elbow
    double forearm{0.11};          // L3: elbow -> gripper tip

    JointLimits base{0.0, 180.0};
    JointLimits shoulder{30.0, 150.0};
    JointLimits elbow{30.0, 150.0};
    JointLimits gripper{60.0, 120.0};

    /// Elbow servo mounted the other way round (same as deploy.py --flip-elbow).
    bool elbow_inverted{false};
};

[[nodiscard]] JointAngles clampToLimits(const JointAngles& q, const ArmConfig& cfg);
[[nodiscard]] bool withinLimits(const JointAngles& q, const ArmConfig& cfg, double tol = 1e-9);

}  // namespace armvision
