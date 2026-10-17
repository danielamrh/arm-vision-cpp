# Firmware — ESP32-S3 Robot Arm Controller

PlatformIO firmware for the Spacnana SNAM1500 4-DOF robot arm.

## Hardware Setup

| Servo | Joint | GPIO |
|-------|-------|------|
| Servo 0 | Base (left/right) | GPIO 4 |
| Servo 1 | Shoulder (up/down) | GPIO 5 |
| Servo 2 | Elbow (bend) | GPIO 6 |
| Servo 3 | Gripper (open/close) | GPIO 7 |

**Power:** Servos require a separate 5V supply — do not power from the ESP32 3.3V pin.

**IMU (planned — MPU-6050):**
| Pin | GPIO |
|-----|------|
| SDA | GPIO 8 |
| SCL | GPIO 9 |
| VCC | 3.3V |
| GND | GND |

## Requirements

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- Board: `esp32-s3-devkitc-1`

## Build & Flash

```bash
pio run --target upload
```

Or use the PlatformIO VS Code extension: **Build** → **Upload**.

## Debug

JTAG is available via the built-in USB interface (no external debugger needed):

```
debug_tool = esp-builtin
```

## Current Behavior

Serial control over the **native USB port** (the one used for flashing, USB-Serial/JTAG,
VID 303A PID 1001). `ARDUINO_USB_CDC_ON_BOOT=1` in `platformio.ini` routes `Serial` there;
without it `Serial` goes to UART0 and the host never gets an answer.

On startup all servos move to 90° and the board prints `READY`. Then it accepts
`"<base>,<shoulder>,<elbow>,<gripper>
"` (integer degrees, clamped to the joint limits),
moves, and answers `OK` (or `ERR: bad format: ...`).

## Notes

- `servo.detach()` is called after each move to eliminate jitter when the servo is idle.
- All angles are in degrees (0–180).
- Home position: `[90, 90, 90, 90]`
