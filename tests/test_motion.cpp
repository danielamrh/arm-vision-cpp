#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "armvision/kinematics.hpp"
#include "armvision/motion.hpp"

namespace armvision {
namespace {

double largestJointStep(const JointAngles& a, const JointAngles& b) {
    return std::max({std::abs(a.base - b.base), std::abs(a.shoulder - b.shoulder),
                     std::abs(a.elbow - b.elbow), std::abs(a.gripper - b.gripper)});
}

// ── Lesson 1: safety ─────────────────────────────────────────

TEST(Motion, HomeIsSafe) {
    EXPECT_EQ(findViolation(JointAngles{}, ArmConfig{}), std::nullopt);
}

TEST(Motion, DetectsJointOutsideLimits) {
    EXPECT_EQ(findViolation({90.0, 20.0, 90.0, 90.0}, ArmConfig{}), Violation::JointLimit);
}

TEST(Motion, DetectsTipBelowTable) {
    // Shoulder 40° points the stretched arm 50° downwards: the tip ends up under the table.
    EXPECT_EQ(findViolation({90.0, 40.0, 90.0, 90.0}, ArmConfig{}), Violation::TipBelowTable);
}

TEST(Motion, DetectsElbowBelowSafetyMargin) {
    // Upper arm 30° down puts the elbow 3 cm above the table; the forearm bent 60° up keeps
    // the tip at 8.5 cm. With a 5 cm margin only the elbow is too low.
    ArmConfig cfg;
    cfg.min_height = 0.05;
    const JointAngles q{90.0, 60.0, 150.0, 90.0};
    ASSERT_GT(forwardKinematics(q, cfg).z(), cfg.min_height);
    EXPECT_EQ(findViolation(q, cfg), Violation::ElbowBelowTable);
}

TEST(Motion, ViolationsHaveReadableNames) {
    EXPECT_FALSE(toString(Violation::JointLimit).empty());
    EXPECT_NE(toString(Violation::TipBelowTable), toString(Violation::ElbowBelowTable));
}

// ── Lesson 2: joint-space interpolation ──────────────────────

TEST(Motion, InterpolateJointsSplitsTheLargestMoveIntoEqualSteps) {
    const JointAngles from{90.0, 90.0, 90.0, 90.0};
    const JointAngles to{120.0, 60.0, 90.0, 90.0};
    const auto path = interpolateJoints(from, to, 10.0);
    ASSERT_EQ(path.size(), 3u);
    EXPECT_EQ(path[0], (JointAngles{100.0, 80.0, 90.0, 90.0}));
    EXPECT_EQ(path[1], (JointAngles{110.0, 70.0, 90.0, 90.0}));
    EXPECT_EQ(path[2], to);
}

TEST(Motion, InterpolateJointsNeverExceedsTheMaximumStep) {
    const JointAngles from{10.0, 140.0, 35.0, 60.0};
    const JointAngles to{170.0, 45.0, 140.0, 120.0};
    const auto path = interpolateJoints(from, to, 7.0);
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.back(), to);
    JointAngles previous = from;
    for (const auto& q : path) {
        EXPECT_LE(largestJointStep(previous, q), 7.0 + 1e-9);
        previous = q;
    }
}

TEST(Motion, InterpolateJointsIsEmptyWithoutMovement) {
    EXPECT_TRUE(interpolateJoints(JointAngles{}, JointAngles{}, 5.0).empty());
}

TEST(Motion, InterpolateJointsRejectsNonPositiveStep) {
    EXPECT_THROW((void)interpolateJoints(JointAngles{}, {100.0, 90.0, 90.0, 90.0}, 0.0),
                 std::invalid_argument);
}

// ── Lesson 3: easing ─────────────────────────────────────────

static_assert(Easing<decltype(&linear)>);
static_assert(!Easing<int>);

TEST(Motion, SmoothstepStartsAndStopsGently) {
    EXPECT_DOUBLE_EQ(smoothstep(0.0), 0.0);
    EXPECT_DOUBLE_EQ(smoothstep(0.5), 0.5);
    EXPECT_DOUBLE_EQ(smoothstep(1.0), 1.0);
    EXPECT_LT(smoothstep(0.01), 0.01);        // slower than linear at the start ...
    EXPECT_GT(smoothstep(0.99), 0.99);        // ... and at the end
}

TEST(Motion, EasedInterpolationWithLinearIsEvenlySpaced) {
    const JointAngles to{130.0, 90.0, 90.0, 90.0};
    const auto path = interpolateEased(JointAngles{}, to, 4, linear);
    ASSERT_EQ(path.size(), 4u);
    EXPECT_DOUBLE_EQ(path[0].base, 100.0);
    EXPECT_DOUBLE_EQ(path[3].base, 130.0);
}

TEST(Motion, SmoothstepMakesSmallStepsAtBothEnds) {
    const JointAngles from{};
    const JointAngles to{170.0, 90.0, 90.0, 90.0};
    const auto path = interpolateEased(from, to, 10, smoothstep);
    ASSERT_EQ(path.size(), 10u);
    const double first = path[0].base - from.base;
    const double middle = path[5].base - path[4].base;
    const double last = path[9].base - path[8].base;
    EXPECT_LT(first, middle / 2);
    EXPECT_LT(last, middle / 2);
    EXPECT_EQ(path.back(), to);
}

TEST(Motion, EasedInterpolationAcceptsLambdas) {
    const auto quadratic = [](double t) { return t * t; };
    const auto path = interpolateEased(JointAngles{}, {90.0, 90.0, 90.0, 130.0}, 2, quadratic);
    ASSERT_EQ(path.size(), 2u);
    EXPECT_DOUBLE_EQ(path[0].gripper, 90.0 + 0.25 * 40.0);
}

// ── Lesson 4: straight lines ─────────────────────────────────

TEST(Motion, StraightLineKeepsTheTipOnTheLine) {
    const ArmConfig cfg;
    const JointAngles from{60.0, 100.0, 70.0, 90.0};
    const Eigen::Vector3d start = forwardKinematics(from, cfg);
    const Eigen::Vector3d target = forwardKinematics({80.0, 100.0, 70.0, 90.0}, cfg);

    const auto path = straightLine(from, target, cfg, 0.005);
    ASSERT_TRUE(path.has_value());
    ASSERT_GE(path->size(), 2u);
    EXPECT_LT((forwardKinematics(path->back(), cfg) - target).norm(), 1e-6);

    const Eigen::Vector3d direction = (target - start).normalized();
    JointAngles previous = from;
    for (const auto& q : *path) {
        const Eigen::Vector3d p = forwardKinematics(q, cfg);
        const Eigen::Vector3d offset = p - start;
        const double off_line = (offset - offset.dot(direction) * direction).norm();
        EXPECT_LT(off_line, 1e-6);                      // on the line
        EXPECT_LT(largestJointStep(previous, q), 5.0);  // no jumps between solutions
        previous = q;
    }
}

TEST(Motion, StraightLineFailsForUnreachableTarget) {
    EXPECT_FALSE(straightLine(JointAngles{}, {0.5, 0.0, 0.1}, ArmConfig{}, 0.01).has_value());
}

TEST(Motion, StraightLineFailsIfItWouldHitTheTable) {
    const ArmConfig cfg;
    const Eigen::Vector3d below_table = forwardKinematics({90.0, 40.0, 90.0, 90.0}, cfg);
    ASSERT_LT(below_table.z(), 0.0);
    EXPECT_FALSE(straightLine(JointAngles{}, below_table, cfg, 0.01).has_value());
}

}  // namespace
}  // namespace armvision
