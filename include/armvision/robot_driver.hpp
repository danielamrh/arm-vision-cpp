#pragma once

#include <chrono>
#include <ostream>
#include <string>

#include "armvision/arm_config.hpp"
#include "armvision/serial_port.hpp"

namespace armvision {

/// Hardware abstraction: everything above this interface (vision, planning) does not know
/// which arm it is talking to. A future SO-101 / bus-servo arm only needs a new driver.
class RobotDriver {
public:
    virtual ~RobotDriver() = default;

    /// Moves to `q` (clamped to the joint limits) and blocks until the move is acknowledged.
    virtual void moveTo(const JointAngles& q) = 0;

    /// Last commanded angles after clamping. SG90 servos have no feedback,
    /// so this is what we asked for, not a measurement.
    [[nodiscard]] virtual JointAngles commanded() const = 0;
};

/// Driver for the ESP32 firmware in firmware/src/main.cpp.
class SerialArmDriver final : public RobotDriver {
public:
    struct Options {
        unsigned baud{115200};
        std::chrono::milliseconds ready_timeout{3000};  // wait for "READY" after opening
        std::chrono::milliseconds ack_timeout{2000};    // wait for "OK" after a command
    };

    SerialArmDriver(const std::string& port, ArmConfig cfg, Options opts);
    SerialArmDriver(const std::string& port, ArmConfig cfg)
        : SerialArmDriver(port, cfg, Options{}) {}

    void moveTo(const JointAngles& q) override;
    [[nodiscard]] JointAngles commanded() const override { return commanded_; }

private:
    void waitForReady();

    SerialPort port_;
    ArmConfig cfg_;
    Options opts_;
    JointAngles commanded_{};  // firmware homes to 90/90/90/90 on boot
};

/// Hardware-free driver for tests and dry runs. Optionally logs every command.
class SimDriver final : public RobotDriver {
public:
    explicit SimDriver(ArmConfig cfg, std::ostream* log = nullptr) : cfg_(cfg), log_(log) {}

    void moveTo(const JointAngles& q) override;
    [[nodiscard]] JointAngles commanded() const override { return commanded_; }

private:
    ArmConfig cfg_;
    std::ostream* log_;
    JointAngles commanded_{};
};

}  // namespace armvision
