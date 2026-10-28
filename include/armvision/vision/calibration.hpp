#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

namespace armvision::vision {

/// Geometry of a printed ChArUco board (chessboard with ArUco markers in the white squares).
/// Uses a different dictionary than the markers on the arm, so the two never get mixed up.
struct CharucoBoardSpec {
    int squares_x{7};
    int squares_y{5};
    float square_length{0.030f};  // metres: measure on the print, printers scale!
    float marker_length{0.022f};  // metres
    cv::aruco::PredefinedDictionaryType dictionary{cv::aruco::DICT_5X5_100};
};

[[nodiscard]] cv::aruco::CharucoBoard makeCharucoBoard(const CharucoBoardSpec& spec);

/// Result of a camera calibration (pinhole model with radial/tangential distortion).
struct CameraIntrinsics {
    cv::Size image_size;
    cv::Mat camera_matrix;               // 3x3, CV_64F: [fx 0 cx; 0 fy cy; 0 0 1] in pixels
    cv::Mat dist_coeffs;                 // 1x5, CV_64F: k1 k2 p1 p2 k3
    double rms_reprojection_error{0.0};  // pixels; < 0.5 is good, > 1 is suspicious

    /// Writes a YAML/XML file (format chosen by the extension, e.g. "camera.yaml").
    void save(const std::string& path) const;
    /// Throws std::runtime_error if the file cannot be read or is incomplete.
    [[nodiscard]] static CameraIntrinsics load(const std::string& path);
};

/// Collects views of a ChArUco board and calibrates the camera from them.
class CharucoCalibrator {
public:
    static constexpr std::size_t kMinCornersPerView = 8;
    static constexpr std::size_t kMinViews = 5;

    explicit CharucoCalibrator(const CharucoBoardSpec& spec = {});

    /// Detects the board in `image` and keeps its corners if enough were found.
    /// Returns false (and keeps nothing) if not. All images must have the same size,
    /// otherwise std::invalid_argument is thrown.
    bool addView(const cv::Mat& image);

    [[nodiscard]] std::size_t viewCount() const { return object_points_.size(); }

    /// Runs the calibration. Throws std::runtime_error with fewer than kMinViews views.
    [[nodiscard]] CameraIntrinsics calibrate() const;

private:
    cv::aruco::CharucoBoard board_;
    cv::aruco::CharucoDetector detector_;
    cv::Size image_size_;
    std::vector<std::vector<cv::Point3f>> object_points_;  // board corners, metres, z = 0
    std::vector<std::vector<cv::Point2f>> image_points_;   // the same corners in the image
};

}  // namespace armvision::vision
