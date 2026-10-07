#include "armvision/arm_config.hpp"

#include <algorithm>

namespace armvision {

namespace {

double clamp(double v, const JointLimits& lim) { return std::clamp(v, lim.min, lim.max); }

bool inside(double v, const JointLimits& lim, double tol) {
    return v >= lim.min - tol && v <= lim.max + tol;
}

}  // namespace

JointAngles clampToLimits(const JointAngles& q, const ArmConfig& cfg) {
    return {clamp(q.base, cfg.base), clamp(q.shoulder, cfg.shoulder), clamp(q.elbow, cfg.elbow),
            clamp(q.gripper, cfg.gripper)};
}

bool withinLimits(const JointAngles& q, const ArmConfig& cfg, double tol) {
    return inside(q.base, cfg.base, tol) && inside(q.shoulder, cfg.shoulder, tol) &&
           inside(q.elbow, cfg.elbow, tol) && inside(q.gripper, cfg.gripper, tol);
}

}  // namespace armvision
