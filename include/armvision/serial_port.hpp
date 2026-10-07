#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace armvision {

/// Minimal blocking serial port (8N1), RAII-owned.
/// The OS-specific parts live behind a pImpl so <windows.h> does not leak into headers.
/// Currently implemented for Windows only; the POSIX (termios) version is an open exercise.
class SerialPort {
public:
    /// `port` is e.g. "COM3". Throws std::system_error if the port cannot be opened.
    SerialPort(const std::string& port, unsigned baud);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&) noexcept;
    SerialPort& operator=(SerialPort&&) noexcept;

    void write(std::string_view data);

    /// Reads one line ('\n'-terminated, '\r' and '\n' stripped).
    /// Returns std::nullopt if no complete line arrived within `timeout`.
    [[nodiscard]] std::optional<std::string> readLine(std::chrono::milliseconds timeout);

    /// Drops everything that was received but not read yet.
    void discardInput();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace armvision
