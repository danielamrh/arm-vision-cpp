#include "armvision/serial_port.hpp"

// Win32 implementation of SerialPort. The POSIX one is in serial_port_posix.cpp;
// CMake compiles exactly one of the two.

#include <stdexcept>
#include <system_error>

#include <windows.h>

namespace armvision {

namespace {

[[noreturn]] void throwLastError(const std::string& what) {
    throw std::system_error(static_cast<int>(GetLastError()), std::system_category(), what);
}

}  // namespace

struct SerialPort::Impl {
    HANDLE handle{INVALID_HANDLE_VALUE};
    std::string pending;  // bytes received after the last complete line

    ~Impl() {
        if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle);
    }
};

SerialPort::SerialPort(const std::string& port, unsigned baud) : impl_(std::make_unique<Impl>()) {
    // The \\.\ prefix is required for COM10 and above and harmless below.
    const std::string path = R"(\\.\)" + port;
    impl_->handle = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                                OPEN_EXISTING, 0, nullptr);
    if (impl_->handle == INVALID_HANDLE_VALUE) throwLastError("cannot open " + port);

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(impl_->handle, &dcb)) throwLastError("GetCommState");
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    // Assert DTR and RTS like pyserial does, which is what deploy.py was tested with.
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(impl_->handle, &dcb)) throwLastError("SetCommState");

    // ReadFile returns as soon as at least one byte is available, or after 50 ms with none.
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutConstant = 1000;
    if (!SetCommTimeouts(impl_->handle, &timeouts)) throwLastError("SetCommTimeouts");
}

void SerialPort::write(std::string_view data) {
    DWORD written = 0;
    if (!WriteFile(impl_->handle, data.data(), static_cast<DWORD>(data.size()), &written,
                   nullptr)) {
        throwLastError("WriteFile");
    }
    if (written != data.size()) throw std::runtime_error("serial write timed out");
}

std::optional<std::string> SerialPort::readLine(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::string& pending = impl_->pending;

    while (true) {
        if (const auto nl = pending.find('\n'); nl != std::string::npos) {
            std::string line = pending.substr(0, nl);
            pending.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return line;
        }
        if (std::chrono::steady_clock::now() >= deadline) return std::nullopt;

        char buf[256];
        DWORD n = 0;
        if (!ReadFile(impl_->handle, buf, sizeof(buf), &n, nullptr)) throwLastError("ReadFile");
        pending.append(buf, n);
    }
}

void SerialPort::discardInput() {
    PurgeComm(impl_->handle, PURGE_RXCLEAR);
    impl_->pending.clear();
}

// Defined here, where Impl is a complete type.
SerialPort::~SerialPort() = default;
SerialPort::SerialPort(SerialPort&&) noexcept = default;
SerialPort& SerialPort::operator=(SerialPort&&) noexcept = default;

}  // namespace armvision
