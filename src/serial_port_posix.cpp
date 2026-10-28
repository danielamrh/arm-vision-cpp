#include "armvision/serial_port.hpp"

// POSIX (Linux / Raspberry Pi) implementation of SerialPort, using termios.
// The Win32 one is in serial_port_win.cpp; CMake compiles exactly one of the two.
//
// Written step by step in docs/tutorials/01-posix-serial-port.md.
// tests/test_serial_posix.cpp checks it against a pseudo-terminal (no hardware needed):
//   Windows PC (WSL):  wsl bash tools/test_serial_wsl.sh
//   Linux / Pi:        ctest --preset linux -R SerialPortPosix
//
// Useful man pages:  man 2 open,  man 3 termios,  man 2 poll,  man 2 read,  man 2 write

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>

namespace armvision {

namespace {

/// Throws std::system_error for the current errno, e.g. "cannot open /dev/ttyACM0: No such file".
[[noreturn]] void throwErrno(const std::string& what) {
    throw std::system_error(errno, std::generic_category(), what);
}

/// Maps a baud rate in bit/s to the termios constant, or nullopt if unsupported.
std::optional<speed_t> toSpeed(unsigned baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return std::nullopt;
    }
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
    // Reject bad arguments before touching the device ("fail fast").
    const auto speed = toSpeed(baud);
    if (!speed) throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud));

    // O_NOCTTY: the port must not become our controlling terminal.
    impl_->fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (impl_->fd < 0) throwErrno("cannot open " + port);
    // From here on Impl owns the descriptor and closes it, even if we throw below.

    termios tty{};
    if (::tcgetattr(impl_->fd, &tty) != 0) throwErrno("cannot get termios for " + port);

    ::cfmakeraw(&tty);              // no echo, no line editing, no "\n" -> "\r\n"
    tty.c_cflag |= CLOCAL | CREAD;  // ignore modem lines, enable the receiver
    tty.c_cc[VMIN] = 0;             // read() never blocks; readLine() waits with poll()
    tty.c_cc[VTIME] = 0;
    if (::cfsetispeed(&tty, *speed) != 0 || ::cfsetospeed(&tty, *speed) != 0) {
        throwErrno("cannot set baud rate for " + port);
    }

    if (::tcsetattr(impl_->fd, TCSANOW, &tty) != 0) throwErrno("cannot set termios for " + port);
    if (::tcflush(impl_->fd, TCIOFLUSH) != 0) throwErrno("cannot flush " + port);
}

void SerialPort::write(std::string_view data) {
    // ::write() may write fewer bytes than asked for, so loop until everything is out.
    while (!data.empty()) {
        const ssize_t result = ::write(impl_->fd, data.data(), data.size());
        if (result < 0) {
            if (errno == EINTR) continue;  // interrupted by a signal: just try again
            throwErrno("write");
        }
        data.remove_prefix(static_cast<std::size_t>(result));  // result >= 0 here
    }
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

        // Wait until data arrives or the deadline passes. Round the remaining time up:
        // rounding down would turn e.g. 0.7 ms into poll(..., 0) and give up too early.
        const auto remaining_ms = std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
        pollfd pfd{impl_->fd, POLLIN, 0};
        const int poll_result = ::poll(&pfd, 1, static_cast<int>(remaining_ms.count()));
        if (poll_result == 0) return std::nullopt;  // timeout
        if (poll_result < 0) {
            if (errno == EINTR) continue;  // interrupted by a signal: wait again
            throwErrno("poll");
        }

        char buf[256];
        const ssize_t read_result = ::read(impl_->fd, buf, sizeof(buf));
        if (read_result < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            throwErrno("read");
        }
        pending.append(buf, static_cast<std::size_t>(read_result));  // 0 bytes: nothing new
    }
}

void SerialPort::discardInput() {
    // Unread input lives in two places: the kernel's buffer and our own `pending`.
    if (::tcflush(impl_->fd, TCIFLUSH) != 0) throwErrno("tcflush");
    impl_->pending.clear();
}

// Defined here, where Impl is a complete type.
SerialPort::~SerialPort() = default;
SerialPort::SerialPort(SerialPort&&) noexcept = default;
SerialPort& SerialPort::operator=(SerialPort&&) noexcept = default;

}  // namespace armvision
