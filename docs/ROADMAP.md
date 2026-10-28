# Roadmap

Each milestone produces something that runs on the real arm, plus a set of C++ topics it
practises.

## M0 — Scaffold ✅

- CMake project with presets, FetchContent (Eigen, GoogleTest), optional OpenCV
- Forward / closed-form inverse kinematics, protocol, Win32 serial port, `RobotDriver` interface
- Unit tests, `arm_cli` with `--sim`

*C++:* CMake targets and visibility, RAII, pImpl, rule of five, `std::optional`,
`std::from_chars`, `std::format`, virtual interfaces, GoogleTest.

## M1 — Hardware loop

- Drive the real arm with `arm_cli` and check the serial driver (READY handling, timeouts)
- Check the angle conventions: does `shoulder > 90` really lift the arm, is the elbow reversed?
- **Workspace:** with the current limits (shoulder/elbow 30–150°) the tip can only reach points
  18.2–21 cm from the shoulder axis, a thin spherical shell. Measure the real mechanical
  limits of each joint and widen `ArmConfig` (and the firmware limits) where safe.
- Add a table-collision check (tip must stay above z = 0) to the IK / driver
- Proper 5 V / 4 A servo supply; check whether the shoulder sags under gravity once the
  servos are detached (if so, keep shoulder and elbow attached)

*C++:* error handling with exceptions vs. `std::expected`-style results, `std::chrono`.

## M1.5 — Raspberry Pi as the robot computer

- Raspberry Pi OS Lite 64-bit (Trixie), headless, SSH; build the project natively on the Pi
- POSIX `SerialPort` with termios, tested against a pseudo-terminal
  (exercise: `src/serial_port_posix.cpp`, tests in `tests/test_serial_posix.cpp`)
- `arm_cli --port /dev/ttyACM0` on the Pi
- Camera Module 3 via libcamera: grab frames into `cv::Mat` (libcamera C++ API or a GStreamer
  `libcamerasrc` pipeline), lock the autofocus to a fixed lens position
- No display on the Pi: save frames / stream them to the PC instead of `cv::imshow`

*C++:* platform-specific code behind one interface, POSIX APIs (`open`, `termios`, `poll`),
errno and `std::system_error`, test doubles (pty), CMake per platform.

## M2 — Camera calibration

- `calibrate_camera` app: ChArUco board, `cv::calibrateCamera`, save intrinsics to YAML
  (exercise: `src/vision/calibration.cpp`, guide: `docs/tutorials/02-camera-calibration.md`)
- Undistortion and reprojection error report
- Calibrate the Pi camera once it is mounted and its focus is locked

*C++:* `cv::FileStorage`, value types for calibration data, small CLI argument parser.

## M3 — Where is the arm? (extrinsics + kinematic calibration)

- Marker on the table → camera-to-base transform (`cv::solvePnP`, Eigen `Isometry3d`)
- Marker on the gripper → measure the tip position for many joint configurations
- Fit link lengths and servo offsets by nonlinear least squares (Gauss-Newton with Eigen) and
  compare predicted vs. measured tip position, before and after the fit

*C++:* Eigen geometry module, templates / functors for the residual, numerical Jacobians.

## M4 — Closed-loop visual servoing

- Camera thread → lock-free or mutex-protected latest-frame buffer → control thread
- Position-based servoing: measure the tip error in the base frame, correct via IK
- Measure latency and accuracy (open loop vs. closed loop)

*C++:* `std::jthread`, `std::stop_token`, `std::mutex` / `std::condition_variable`, atomics,
profiling.

## M5 — Pick and place

- Detect objects on the table (colour segmentation first), back-project onto the table plane
- State machine: approach → descend → grasp → lift → place
- Success-rate evaluation over N trials

*C++:* `std::variant`-based state machine, logging, config files.

## M6 — ROS 2 integration

Wrap the existing libraries in ROS 2 (Jazzy) nodes instead of rewriting them. The core
libraries stay ROS-free, so `arm_cli` and the tests keep working without ROS.

- Platform: ROS 2 has no binary packages for Raspberry Pi OS and the Pi 3 has only 1 GB RAM, so
  run ROS 2 in a Docker container (`ros:jazzy`, arm64) on the Pi, or run the heavy parts
  (planning, RViz) on the PC (Ubuntu 24.04 / WSL) with the Pi as hardware node
- `arm_driver` node around `RobotDriver`: subscribes to joint commands, publishes the
  commanded joint states (`sensor_msgs/JointState`)
- `camera` + `marker_detector` nodes: publish images and marker poses; camera calibration
  as `sensor_msgs/CameraInfo`
- URDF of the arm (link lengths from the M3 calibration), `robot_state_publisher`, TF tree
  (camera ↔ base ↔ tip), visualisation in RViz
- Launch file that starts everything; optional: MoveIt 2 for motion planning
- Optional: micro-ROS on the ESP32, so the firmware talks to ROS directly

*C++:* `rclcpp` (nodes, publishers/subscribers, timers, parameters), callbacks and executors,
`colcon` / `ament_cmake` packaging next to plain CMake, keeping library code independent of the
middleware.

## Later / optional

- ONNX Runtime object detector instead of colour thresholds
- Firmware v2: binary protocol with CRC, smooth trajectories generated on the ESP32
- Swap in a bus-servo arm (e.g. SO-101) by writing a second `RobotDriver`
