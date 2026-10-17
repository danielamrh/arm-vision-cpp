// arm_cli — move the arm from the command line.
//
//   arm_cli [--port COM3 | --sim] [--flip-elbow] [--no-flip-shoulder] home
//   arm_cli ... joints <base> <shoulder> <elbow> <gripper>   model degrees (see arm_config.hpp)
//   arm_cli ... xyz <x> <y> <z>                              metres, base frame (uses IK)
//   arm_cli fk <base> <shoulder> <elbow>                     print tip position, no hardware

#include <cstdlib>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "armvision/kinematics.hpp"
#include "armvision/robot_driver.hpp"

namespace {

using armvision::ArmConfig;
using armvision::JointAngles;

int usage() {
    std::cerr << "usage:\n"
                 "  arm_cli [--port COM3 | --sim] [--flip-elbow] home\n"
                 "  arm_cli [--port COM3 | --sim] [--flip-elbow] joints <b> <s> <e> <g>\n"
                 "  arm_cli [--port COM3 | --sim] [--flip-elbow] xyz <x> <y> <z>\n"
                 "  arm_cli fk <b> <s> <e>\n";
    return EXIT_FAILURE;
}

std::vector<double> toNumbers(std::span<const std::string_view> args) {
    std::vector<double> v;
    for (auto a : args) v.push_back(std::stod(std::string{a}));
    return v;
}

void printTip(const JointAngles& q, const ArmConfig& cfg) {
    const auto p = armvision::forwardKinematics(q, cfg);
    std::cout << "tip [m]: x=" << p.x() << " y=" << p.y() << " z=" << p.z() << '\n';
}

}  // namespace

int main(int argc, char** argv) try {
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    ArmConfig cfg;
    std::string port;
    bool sim = false;

    std::size_t i = 0;
    for (; i < args.size() && args[i].starts_with("--"); ++i) {
        if (args[i] == "--sim") {
            sim = true;
        } else if (args[i] == "--flip-elbow") {
            cfg.elbow_inverted = true;
        } else if (args[i] == "--no-flip-shoulder") {
            cfg.shoulder_inverted = false;
        } else if (args[i] == "--port" && i + 1 < args.size()) {
            port = args[++i];
        } else {
            return usage();
        }
    }
    if (i >= args.size()) return usage();

    const std::string_view cmd = args[i];
    const auto nums = toNumbers(std::span(args).subspan(i + 1));

    if (cmd == "fk") {
        if (nums.size() != 3) return usage();
        printTip({nums[0], nums[1], nums[2], 90.0}, cfg);
        return EXIT_SUCCESS;
    }

    if (!sim && port.empty()) {
        std::cerr << "need --port <COMx> or --sim\n";
        return EXIT_FAILURE;
    }
    std::unique_ptr<armvision::RobotDriver> arm;
    if (sim) {
        arm = std::make_unique<armvision::SimDriver>(cfg, &std::cout);
    } else {
        arm = std::make_unique<armvision::SerialArmDriver>(port, cfg);
    }

    JointAngles target;
    if (cmd == "home" && nums.empty()) {
        target = JointAngles{};
    } else if (cmd == "joints" && nums.size() == 4) {
        target = {nums[0], nums[1], nums[2], nums[3]};
    } else if (cmd == "xyz" && nums.size() == 3) {
        const auto q = armvision::inverseKinematics({nums[0], nums[1], nums[2]}, cfg);
        if (!q) {
            std::cerr << "target not reachable within the joint limits\n";
            return EXIT_FAILURE;
        }
        target = *q;
    } else {
        return usage();
    }

    arm->moveTo(target);
    const auto q = arm->commanded();
    std::cout << "commanded: " << q.base << ", " << q.shoulder << ", " << q.elbow << ", "
              << q.gripper << '\n';
    printTip(q, cfg);
    return EXIT_SUCCESS;
} catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
}
