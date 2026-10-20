# arm-vision-cpp

Camera-guided control of a low-cost 4-DOF robot arm, written in modern C++ (C++20).
It's a learning project: the goal is clean, tested C++ around a real robot, starting with
kinematics and serial control and ending with closed-loop visual servoing and pick-and-place.

**Hardware**
- Spacnana SNAM1500 arm with 4× SG90 servos (base, shoulder, elbow, gripper)
- ESP32-S3 running [`firmware/src/main.cpp`](firmware/src/main.cpp) (text protocol, 115200 baud)
- Raspberry Pi 3 with Camera Module 3 (target), Windows PC (development), printed ArUco markers

## Layout

```
include/armvision/      public headers
  arm_config.hpp        joint angles, limits, link lengths
  kinematics.hpp        forward kinematics + closed-form inverse kinematics (Eigen)
  protocol.hpp          encode/parse the firmware's text protocol
  serial_port.hpp       RAII serial port (pImpl; Win32 and POSIX/termios implementations)
  robot_driver.hpp      RobotDriver interface, SerialArmDriver, SimDriver
  vision/               OpenCV code (only built if OpenCV is found)
src/                    implementations
apps/
  arm_cli               move the arm from the command line (or --sim)
  camera_view           live webcam view with ArUco detection     [OpenCV]
  make_markers          print-ready ArUco markers                 [OpenCV]
tests/                  GoogleTest unit tests
firmware/               ESP32-S3 firmware (PlatformIO, Arduino framework)
docs/ROADMAP.md         milestones and what each one teaches
```

Everything above `RobotDriver` is hardware-agnostic. A different arm only needs a new driver.

## Build (Windows, Visual Studio 2022 / Build Tools)

CMake ships with Visual Studio. If `cmake` is not on your `PATH`, use a *Developer PowerShell
for VS 2022*. Eigen and GoogleTest are downloaded automatically on first configure.

```powershell
cmake --preset msvc
cmake --build --preset debug
ctest --preset debug
```

If configuring fails with `LNK1104: MSVCRTD.lib`, an incomplete MSVC toolset is being picked.
In that case, pin a complete toolset in a local `CMakeUserPresets.json` (not committed). See
the one used on the development machine:
`"toolset": { "value": "v143,version=14.34.17.4", "strategy": "set" }`.

## Build (Linux / Raspberry Pi OS)

```bash
sudo apt install -y build-essential cmake git libopencv-dev libeigen3-dev libgtest-dev
cmake --preset linux
cmake --build --preset linux
ctest --preset linux
```

On Linux, Eigen, GoogleTest and OpenCV come from the system packages. The ESP32 shows up as
`/dev/ttyACM0`; add yourself to the `dialout` group to use it without `sudo`.
Needs GCC 13+ (`std::format`) and OpenCV 4.7+, e.g. Raspberry Pi OS based on Debian 13 (Trixie).

### Optional on Windows: OpenCV (for the vision apps)

Install the official Windows release of OpenCV ≥ 4.7 (ArUco lives in `objdetect` since 4.7),
e.g. extract it to `C:\opencv`, then configure with:

```powershell
cmake --preset msvc -DOpenCV_DIR=C:/opencv/build
```

The OpenCV DLLs are copied next to the executables automatically.

## Usage

```powershell
# no hardware needed
arm_cli fk 60 110 70                    # tip position for servo angles base/shoulder/elbow
arm_cli --sim xyz 0.09 0.155 0.135      # IK, then "send" to the simulated arm

# real arm (servos on their own 5 V supply!)
arm_cli --port COM3 home
arm_cli --port COM3 joints 90 100 80 90
arm_cli --port COM3 xyz 0.09 0.155 0.135
arm_cli --port COM3 --flip-elbow ...    # if the elbow servo is mounted reversed
```

Frame and units: metres, origin at the base centre on the table, z up, x along base angle 0°.

## Status

See [docs/ROADMAP.md](docs/ROADMAP.md). Milestone 0 (scaffold, kinematics, protocol, serial
driver, tests) is done and the arm moves from `arm_cli` on Windows. Vision builds against
OpenCV 4.14; marker detection is unit-tested on synthetic images and works with a webcam.
In progress: porting to the Raspberry Pi (POSIX serial port, Pi camera).

## License

MIT, see [LICENSE](LICENSE).
