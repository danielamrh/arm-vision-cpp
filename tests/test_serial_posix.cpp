// Tests for the POSIX SerialPort against a pseudo-terminal (pty) pair, so no hardware is needed.
// The test plays the role of the ESP32 on the "master" side; SerialPort opens the "slave" side,
// which behaves like a real serial device (/dev/pts/N).

#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <system_error>
#include <thread>

#include "armvision/serial_port.hpp"

namespace armvision {
namespace {

using namespace std::chrono_literals;

/// Owns the master end of a pty; slavePath() is what SerialPort opens.
class PtyPair {
public:
    PtyPair() {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master_ < 0 || ::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
            throw std::system_error(errno, std::generic_category(), "pty setup");
        }
        slave_path_ = ::ptsname(master_);
    }
    ~PtyPair() { ::close(master_); }
    PtyPair(const PtyPair&) = delete;
    PtyPair& operator=(const PtyPair&) = delete;

    [[nodiscard]] const std::string& slavePath() const { return slave_path_; }

    /// Sends bytes as if the ESP32 had written them.
    void send(const std::string& data) const {
        ASSERT_EQ(::write(master_, data.data(), data.size()), static_cast<ssize_t>(data.size()));
    }

    /// Receives up to `n` bytes the SerialPort wrote, waiting at most `timeout`.
    [[nodiscard]] std::string receive(std::size_t n, std::chrono::milliseconds timeout) const {
        std::string out;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (out.size() < n && std::chrono::steady_clock::now() < deadline) {
            pollfd pfd{master_, POLLIN, 0};
            if (::poll(&pfd, 1, 20) <= 0) continue;
            char buf[256];
            const ssize_t got = ::read(master_, buf, sizeof(buf));
            if (got > 0) out.append(buf, static_cast<std::size_t>(got));
        }
        return out;
    }

private:
    int master_{-1};
    std::string slave_path_;
};

TEST(SerialPortPosix, ThrowsSystemErrorForMissingDevice) {
    EXPECT_THROW(SerialPort("/dev/does-not-exist", 115200), std::system_error);
}

TEST(SerialPortPosix, ReadsALineAndStripsCrLf) {
    PtyPair pty;
    SerialPort port(pty.slavePath(), 115200);
    pty.send("OK\r\n");
    const auto line = port.readLine(500ms);
    ASSERT_TRUE(line.has_value());
    EXPECT_EQ(*line, "OK");
}

TEST(SerialPortPosix, ReadLineTimesOutWithoutData) {
    PtyPair pty;
    SerialPort port(pty.slavePath(), 115200);
    const auto start = std::chrono::steady_clock::now();
    EXPECT_FALSE(port.readLine(100ms).has_value());
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_GE(elapsed, 90ms);   // really waited ...
    EXPECT_LT(elapsed, 1000ms);  // ... but did not hang
}

TEST(SerialPortPosix, SplitsSeveralLinesAndKeepsAPartialOne) {
    PtyPair pty;
    SerialPort port(pty.slavePath(), 115200);
    pty.send("READY\nOK\npart");
    EXPECT_EQ(port.readLine(500ms), std::optional<std::string>("READY"));
    EXPECT_EQ(port.readLine(500ms), std::optional<std::string>("OK"));
    EXPECT_FALSE(port.readLine(100ms).has_value());  // "part" has no newline yet
    pty.send("ial\n");
    EXPECT_EQ(port.readLine(500ms), std::optional<std::string>("partial"));
}

TEST(SerialPortPosix, WriteSendsBytesUnchanged) {
    // Fails if the port is not in raw mode: the tty layer would turn "\n" into "\r\n".
    PtyPair pty;
    SerialPort port(pty.slavePath(), 115200);
    port.write("90,120,90,90\n");
    EXPECT_EQ(pty.receive(13, 500ms), "90,120,90,90\n");
}

TEST(SerialPortPosix, DiscardInputDropsBufferedData) {
    PtyPair pty;
    SerialPort port(pty.slavePath(), 115200);
    pty.send("stale\nline\n");
    std::this_thread::sleep_for(50ms);  // let the bytes arrive
    port.discardInput();
    pty.send("fresh\n");
    EXPECT_EQ(port.readLine(500ms), std::optional<std::string>("fresh"));
}

TEST(SerialPortPosix, RejectsUnsupportedBaudRate) {
    PtyPair pty;
    EXPECT_THROW(SerialPort(pty.slavePath(), 12345), std::invalid_argument);
}

}  // namespace
}  // namespace armvision
