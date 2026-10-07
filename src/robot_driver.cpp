#include "armvision/robot_driver.hpp"

#include <format>
#include <stdexcept>

#include "armvision/protocol.hpp"

namespace armvision {

// ── SerialArmDriver ───────────────────────────────────────────

SerialArmDriver::SerialArmDriver(const std::string& port, ArmConfig cfg, Options opts)
    : port_(port, opts.baud), cfg_(cfg), opts_(opts) {
    waitForReady();
}

void SerialArmDriver::waitForReady() {
    // Opening the port usually resets the ESP32; it homes and then prints READY.
    // If the board did not reset, READY never comes, and that is fine too.
    const auto deadline = std::chrono::steady_clock::now() + opts_.ready_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto line = port_.readLine(remaining);
        if (line && protocol::parseReply(*line).type == protocol::ReplyType::Ready) break;
    }
    port_.discardInput();
}

void SerialArmDriver::moveTo(const JointAngles& q) {
    const JointAngles target = clampToLimits(q, cfg_);

    JointAngles wire = target;
    if (cfg_.elbow_inverted) wire.elbow = 180.0 - wire.elbow;
    port_.write(protocol::encodeCommand(wire));

    const auto deadline = std::chrono::steady_clock::now() + opts_.ack_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto line = port_.readLine(remaining);
        if (!line) break;

        const auto reply = protocol::parseReply(*line);
        switch (reply.type) {
            case protocol::ReplyType::Ok:
                commanded_ = target;
                return;
            case protocol::ReplyType::Error:
                throw std::runtime_error("arm rejected command: " + reply.text);
            case protocol::ReplyType::Ready:  // board reset in between, keep waiting
            case protocol::ReplyType::Unknown:
                break;
        }
    }
    throw std::runtime_error(
        std::format("no OK from arm within {} ms", opts_.ack_timeout.count()));
}

// ── SimDriver ─────────────────────────────────────────────────

void SimDriver::moveTo(const JointAngles& q) {
    commanded_ = clampToLimits(q, cfg_);
    if (log_ != nullptr) *log_ << "[sim] " << protocol::encodeCommand(commanded_);
}

}  // namespace armvision
