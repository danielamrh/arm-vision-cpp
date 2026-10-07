#include <gtest/gtest.h>

#include "armvision/kinematics.hpp"

namespace armvision {
namespace {

constexpr double kTol = 1e-9;
// IK goes through acos(), which loses precision near the stretched arm (elbow 90°).
// 1 µm is still far below anything an SG90 servo can resolve.
constexpr double kIkTol = 1e-6;

TEST(ForwardKinematics, HomeIsArmStretchedHorizontallyAlongY) {
    const ArmConfig cfg;
    const auto p = forwardKinematics(JointAngles{}, cfg);  // 90/90/90
    EXPECT_NEAR(p.x(), 0.0, kTol);
    EXPECT_NEAR(p.y(), cfg.upper_arm + cfg.forearm, kTol);
    EXPECT_NEAR(p.z(), cfg.shoulder_height, kTol);
}

TEST(ForwardKinematics, MatchesPythonReferenceValue) {
    // robot_arm_env.forward_kinematics([0, 120, 60]) with L1=8, L2=10, L3=11 cm
    // -> s = 30°, e = -30°: r = 10 cos30 + 11, z = 8 + 10 sin30
    const ArmConfig cfg;
    const auto p = forwardKinematics({0.0, 120.0, 60.0, 90.0}, cfg);
    EXPECT_NEAR(p.x(), 0.10 * std::cos(0.5235987755982988) + 0.11, kTol);
    EXPECT_NEAR(p.y(), 0.0, kTol);
    EXPECT_NEAR(p.z(), 0.08 + 0.05, kTol);
}

TEST(InverseKinematics, RoundTripsOverTheWholeJointSpace) {
    const ArmConfig cfg;
    for (double b = 0.0; b <= 180.0; b += 15.0) {
        for (double s = 30.0; s <= 150.0; s += 10.0) {
            for (double e = 30.0; e <= 150.0; e += 10.0) {
                const JointAngles q{b, s, e, 75.0};
                const auto target = forwardKinematics(q, cfg);

                // With the true angles as reference, IK must give them back exactly.
                const auto sol = inverseKinematics(target, cfg, q);
                ASSERT_TRUE(sol.has_value()) << b << ' ' << s << ' ' << e;
                EXPECT_TRUE(withinLimits(*sol, cfg));
                EXPECT_LT((forwardKinematics(*sol, cfg) - target).norm(), kIkTol);
                EXPECT_NEAR(sol->shoulder, s, 1e-4);
                EXPECT_NEAR(sol->elbow, e, 1e-4);
                EXPECT_DOUBLE_EQ(sol->gripper, 75.0);
            }
        }
    }
}

TEST(InverseKinematics, AnySolutionReachesTheTarget) {
    // With the default (home) reference IK may pick a different branch, but it must still be
    // a valid solution that hits the target.
    const ArmConfig cfg;
    const auto target = forwardKinematics({40.0, 110.0, 70.0, 90.0}, cfg);
    const auto sol = inverseKinematics(target, cfg);
    ASSERT_TRUE(sol.has_value());
    EXPECT_TRUE(withinLimits(*sol, cfg));
    EXPECT_LT((forwardKinematics(*sol, cfg) - target).norm(), kIkTol);
}

TEST(InverseKinematics, RejectsUnreachableTargets) {
    const ArmConfig cfg;
    EXPECT_FALSE(inverseKinematics({0.5, 0.0, 0.1}, cfg).has_value());   // too far
    EXPECT_FALSE(inverseKinematics({0.0, 0.0, 0.08}, cfg).has_value());  // inside the shoulder
}

TEST(InverseKinematics, CannotReachBehindTheBase) {
    // y < 0 would need a base angle outside 0..180°, and with shoulder/elbow limited to
    // 30..150° the arm cannot lean back over the base either.
    const ArmConfig cfg;
    EXPECT_FALSE(inverseKinematics({0.1, -0.1, 0.1}, cfg).has_value());
}

}  // namespace
}  // namespace armvision
