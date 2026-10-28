#include "armvision/vision/calibration.hpp"

// ChArUco camera calibration.
//
// Written step by step in docs/tutorials/02-camera-calibration.md.
// tests/test_calibration.cpp checks it with synthetic images (no camera needed):
//   Windows:  powershell -ExecutionPolicy Bypass -File tools/test.ps1 "Calibration.*"
//   Linux:    ctest --preset linux -R Calibration

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace armvision::vision {

cv::aruco::CharucoBoard makeCharucoBoard(const CharucoBoardSpec& spec) {
    // Chessboard with ArUco markers in the white squares.
    return cv::aruco::CharucoBoard(
        cv::Size(spec.squares_x, spec.squares_y),
        spec.square_length,
        spec.marker_length,
        cv::aruco::getPredefinedDictionary(spec.dictionary)
    );
}

void CameraIntrinsics::save(const std::string& path) const {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);  // YAML or XML, chosen by the extension
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open file for writing: " + path);
    }
    fs << "image_width" << image_size.width << "image_height" << image_size.height;
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    fs << "rms_reprojection_error" << rms_reprojection_error;
    // No fs.release() needed: the destructor closes the file (RAII).
}

CameraIntrinsics CameraIntrinsics::load(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("failed to open file for reading: " + path);
    }

    // Read straight into one result object; missing keys leave the members empty / 0.
    CameraIntrinsics c;
    fs["image_width"] >> c.image_size.width;
    fs["image_height"] >> c.image_size.height;
    fs["camera_matrix"] >> c.camera_matrix;
    fs["dist_coeffs"] >> c.dist_coeffs;
    fs["rms_reprojection_error"] >> c.rms_reprojection_error;

    // A half-read calibration is worse than none: it would silently give wrong measurements.
    if (c.camera_matrix.empty() || c.dist_coeffs.empty() || c.image_size.empty()) {
        throw std::runtime_error("incomplete calibration data in file: " + path);
    }
    return c;
}

CharucoCalibrator::CharucoCalibrator(const CharucoBoardSpec& spec)
    : board_(makeCharucoBoard(spec)), detector_(board_) {}

bool CharucoCalibrator::addView(const cv::Mat& image) {
    // One calibration is valid for exactly one resolution.
    if (!image_size_.empty() && image.size() != image_size_) {
        throw std::invalid_argument("all images must have the same size");
    }

    // detectBoard() returns void: the result is in the output vectors.
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    detector_.detectBoard(image, charuco_corners, charuco_ids);
    if (charuco_corners.size() < kMinCornersPerView) return false;

    // Pair every detected corner with its 3D position on the board (metres, z = 0).
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    board_.matchImagePoints(charuco_corners, charuco_ids, object_points, image_points);

    // One inner list per view: calibrateCamera() estimates a separate board pose for each.
    image_size_ = image.size();
    object_points_.push_back(std::move(object_points));
    image_points_.push_back(std::move(image_points));
    return true;
}

CameraIntrinsics CharucoCalibrator::calibrate() const {
    if (viewCount() < kMinViews) {
        throw std::runtime_error("need at least " + std::to_string(kMinViews) +
                                 " views for calibration, have " + std::to_string(viewCount()));
    }

    // Besides the camera, calibrateCamera() estimates the board pose in every view
    // (rvecs / tvecs); we do not need them here.
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    const double rms = cv::calibrateCamera(object_points_, image_points_, image_size_,
                                           camera_matrix, dist_coeffs, rvecs, tvecs);

    // C++20 designated initializers: every member by name, in declaration order.
    return {.image_size = image_size_,
            .camera_matrix = camera_matrix,
            .dist_coeffs = dist_coeffs,
            .rms_reprojection_error = rms};
}

}  // namespace armvision::vision
