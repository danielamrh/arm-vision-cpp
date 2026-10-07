#include <gtest/gtest.h>

#include <sstream>

#include "armvision/protocol.hpp"
#include "armvision/robot_driver.hpp"

namespace armvision {
namespace {

TEST(Protocol, EncodesIntegerDegreesWithNewline) {
    EXPECT_EQ(protocol::encodeCommand({90.0, 75.4, 110.6, 90.0}), "90,75,111,90\n");
}

TEST(Protocol, ParseCommandIsInverseOfEncode) {
    const JointAngles q{10.0, 45.0, 135.0, 60.0};
    const auto parsed = protocol::parseCommand(protocol::encodeCommand(q));
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(*parsed, q);
}

TEST(Protocol, ParseCommandRejectsMalformedInput) {
    EXPECT_FALSE(protocol::parseCommand("").has_value());
    EXPECT_FALSE(protocol::parseCommand("90,90,90").has_value());
    EXPECT_FALSE(protocol::parseCommand("90,90,90,90,90").has_value());
    EXPECT_FALSE(protocol::parseCommand("90;90;90;90").has_value());
    EXPECT_FALSE(protocol::parseCommand("90,a,90,90").has_value());
}

TEST(Protocol, ClassifiesFirmwareReplies) {
    EXPECT_EQ(protocol::parseReply("OK\r").type, protocol::ReplyType::Ok);
    EXPECT_EQ(protocol::parseReply("READY").type, protocol::ReplyType::Ready);
    EXPECT_EQ(protocol::parseReply("ERR: bad format: x").type, protocol::ReplyType::Error);
    EXPECT_EQ(protocol::parseReply("garbage").type, protocol::ReplyType::Unknown);
    EXPECT_EQ(protocol::parseReply("OK\r\n").text, "OK");
}

TEST(SimDriver, ClampsToJointLimitsAndLogs) {
    std::ostringstream log;
    SimDriver arm(ArmConfig{}, &log);
    arm.moveTo({200.0, 10.0, 90.0, 130.0});
    EXPECT_EQ(arm.commanded(), (JointAngles{180.0, 30.0, 90.0, 120.0}));
    EXPECT_EQ(log.str(), "[sim] 180,30,90,120\n");
}

}  // namespace
}  // namespace armvision
