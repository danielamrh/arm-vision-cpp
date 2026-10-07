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
- Optional: implement the POSIX `SerialPort` (termios)

*C++:* error handling with exceptions vs. `std::expected`-style results, `std::chrono`.

## M2 — Camera calibration

- `calibrate_camera` app: ChArUco board, `cv::calibrateCamera`, save intrinsics to YAML
- Undistortion and reprojection error report

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

## Later / optional

- ONNX Runtime object detector instead of colour thresholds
- Firmware v2: binary protocol with CRC, smooth trajectories generated on the ESP32
- Swap in a bus-servo arm (e.g. SO-101) by writing a second `RobotDriver`
