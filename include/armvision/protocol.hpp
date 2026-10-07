#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "armvision/arm_config.hpp"

/// Text protocol spoken by firmware/src/main.cpp (ESP32-S3, 115200 baud):
///   host -> arm:  "<base>,<shoulder>,<elbow>,<gripper>\n"   integer degrees
///   arm -> host:  "READY" after boot, "OK" after a move, "ERR: ..." on bad input
namespace armvision::protocol {

[[nodiscard]] std::string encodeCommand(const JointAngles& q);

/// Inverse of encodeCommand. Useful for tests and a future firmware simulator.
[[nodiscard]] std::optional<JointAngles> parseCommand(std::string_view line);

enum class ReplyType { Ok, Ready, Error, Unknown };

struct Reply {
    ReplyType type;
    std::string text;  // the line without trailing whitespace
};

[[nodiscard]] Reply parseReply(std::string_view line);

}  // namespace armvision::protocol
