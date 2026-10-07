#include "armvision/protocol.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <format>

namespace armvision::protocol {

namespace {

std::string_view trim(std::string_view s) {
    constexpr std::string_view ws = " \t\r\n";
    const auto first = s.find_first_not_of(ws);
    if (first == std::string_view::npos) return {};
    const auto last = s.find_last_not_of(ws);
    return s.substr(first, last - first + 1);
}

}  // namespace

std::string encodeCommand(const JointAngles& q) {
    return std::format("{},{},{},{}\n", std::lround(q.base), std::lround(q.shoulder),
                       std::lround(q.elbow), std::lround(q.gripper));
}

std::optional<JointAngles> parseCommand(std::string_view line) {
    line = trim(line);
    std::array<int, 4> values{};

    const char* p = line.data();
    const char* end = line.data() + line.size();
    for (std::size_t i = 0; i < values.size(); ++i) {
        const auto [next, ec] = std::from_chars(p, end, values[i]);
        if (ec != std::errc{}) return std::nullopt;
        p = next;
        const bool last = (i + 1 == values.size());
        if (last) {
            if (p != end) return std::nullopt;  // trailing garbage
        } else {
            if (p == end || *p != ',') return std::nullopt;
            ++p;
        }
    }
    return JointAngles{static_cast<double>(values[0]), static_cast<double>(values[1]),
                       static_cast<double>(values[2]), static_cast<double>(values[3])};
}

Reply parseReply(std::string_view line) {
    const std::string_view t = trim(line);
    ReplyType type = ReplyType::Unknown;
    if (t == "OK") {
        type = ReplyType::Ok;
    } else if (t == "READY") {
        type = ReplyType::Ready;
    } else if (t.starts_with("ERR")) {
        type = ReplyType::Error;
    }
    return {type, std::string{t}};
}

}  // namespace armvision::protocol
