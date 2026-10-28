#include "armvision/serial_port.hpp"

// POSIX (Linux / Raspberry Pi) implementation of SerialPort, using termios.
// The Win32 one is in serial_port_win.cpp; CMake compiles exactly one of the two.
//
// ──────────────────────────────────────────────────────────────────────────────
// EXERCISE: the parts marked TODO are left for you to implement.
// Step-by-step guide: docs/tutorials/01-posix-serial-port.md
// tests/test_serial_posix.cpp checks them against a pseudo-terminal (no hardware needed):
//   Windows PC (WSL):  wsl bash tools/test_serial_wsl.sh
//   Linux / Pi:        ctest --preset linux -R SerialPortPosix
//
// Useful man pages:  man 2 open,  man 3 termios,  man 2 poll,  man 2 read,  man 2 write
// ──────────────────────────────────────────────────────────────────────────────

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace armvision {

namespace {

/// Throws std::system_error for the current errno, e.g. "cannot open /dev/ttyACM0: No such file".
[[noreturn]] void throwErrno(const std::string& what) {
    throw std::system_error(errno, std::generic_category(), what);
}

}  // namespace

struct SerialPort::Impl {
    int fd{-1};
    std::string pending;  // bytes received after the last complete line

    ~Impl() {
        if (fd >= 0) ::close(fd);
    }
};

SerialPort::SerialPort(const std::string& port, unsigned baud) : impl_(std::make_unique<Impl>()) {
    // TODO 1: open the device.
    //   - ::open(port.c_str(), ...) with O_RDWR | O_NOCTTY | O_CLOEXEC
    //     (O_NOCTTY: the port must not become our controlling terminal)
    //   - on failure: throwErrno("cannot open " + port)
    //
    // TODO 2: configure the line with termios.
    //   - tcgetattr() to read the current settings
    //   - cfmakeraw(): raw mode, no echo, no line editing, no "\n" -> "\r\n" translation
    //     (the test WriteSendsBytesUnchanged fails if you forget this)
    //   - c_cflag |= CLOCAL | CREAD  (ignore modem lines, enable the receiver)
    //   - c_cc[VMIN] = 0, c_cc[VTIME] = 0  (read() never blocks; we wait with poll() instead)
    //   - baud: map `baud` to a speed_t constant (115200 -> B115200, ...), then
    //     cfsetispeed() + cfsetospeed(). Unsupported rates: throw std::invalid_argument.
    //   - tcsetattr(fd, TCSANOW, ...), then tcflush(fd, TCIOFLUSH) to drop stale bytes
    //   - check every call's return value and use throwErrno() on failure
    (void)port;
    (void)baud;
    throw std::logic_error("SerialPort (POSIX) constructor: not implemented yet");
}

void SerialPort::write(std::string_view data) {
    // TODO 3: write all bytes.
    //   ::write() may write fewer bytes than asked for, so loop until everything is out.
    //   If it fails with errno == EINTR (interrupted by a signal) just try again;
    //   any other error: throwErrno("write").
    (void)data;
    throw std::logic_error("SerialPort::write (POSIX): not implemented yet");
}

std::optional<std::string> SerialPort::readLine(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::string& pending = impl_->pending;

    while (true) {
        // A complete line may already be buffered from an earlier read.
        if (const auto nl = pending.find('\n'); nl != std::string::npos) {
            std::string line = pending.substr(0, nl);
            pending.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return line;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return std::nullopt;

        // TODO 4: wait for data, then read it.
        //   - remaining time = deadline - now, converted to whole milliseconds (int) for poll()
        //   - pollfd{impl_->fd, POLLIN, 0}; ::poll(&pfd, 1, remaining_ms)
        //       returns 0 -> timeout -> return std::nullopt
        //       returns -1 with errno == EINTR -> just continue the loop
        //   - ::read() into a small buffer (e.g. char buf[256]) and append to `pending`
        //       read() returning 0 or -1 with EAGAIN: nothing there, continue
        //       other errors: throwErrno("read")
        (void)now;
        throw std::logic_error("SerialPort::readLine (POSIX): not implemented yet");
    }
}

void SerialPort::discardInput() {
    // TODO 5: drop unread input in the kernel (tcflush with TCIFLUSH) and in `pending`.
    throw std::logic_error("SerialPort::discardInput (POSIX): not implemented yet");
}

// Defined here, where Impl is a complete type.
SerialPort::~SerialPort() = default;
SerialPort::SerialPort(SerialPort&&) noexcept = default;
SerialPort& SerialPort::operator=(SerialPort&&) noexcept = default;

}  // namespace armvision
