#include "armvision/vision/calibration.hpp"

// ChArUco camera calibration.
//
// EXERCISE: the parts marked TODO are left for you to implement, one per lesson.
// Step-by-step guide: docs/tutorials/02-camera-calibration.md
// tests/test_calibration.cpp checks them with synthetic images (no camera needed):
//   Windows:  powershell -ExecutionPolicy Bypass -File tools/test.ps1 "Calibration.*"
//   Linux:    ctest --preset linux -R Calibration

#include <stdexcept>
#include <string>

#include <opencv2/calib3d.hpp>

namespace armvision::vision {

cv::aruco::CharucoBoard makeCharucoBoard(const CharucoBoardSpec& spec) {
    // TODO 1 (lesson 1): build the board from the spec.
    //   cv::aruco::CharucoBoard has a constructor taking
    //   (cv::Size(squares in x, squares in y), square length, marker length, dictionary).
    //   The dictionary comes from cv::aruco::getPredefinedDictionary(spec.dictionary).
    (void)spec;
    throw std::logic_error("makeCharucoBoard: not implemented yet");
}

void CameraIntrinsics::save(const std::string& path) const {
    // TODO 4 (lesson 4): write all four members with cv::FileStorage (mode WRITE).
    //   - throw std::runtime_error if fs.isOpened() is false
    //   - fs << "name" << value << "name2" << value2 ...   (cv::Size: write width and height
    //     separately as "image_width" / "image_height")
    (void)path;
    throw std::logic_error("CameraIntrinsics::save: not implemented yet");
}

CameraIntrinsics CameraIntrinsics::load(const std::string& path) {
    // TODO 4 (lesson 4): the reverse of save().
    //   - cv::FileStorage with mode READ; not opened -> std::runtime_error
    //   - fs["name"] >> member;  for every value written by save()
    //   - if the camera matrix, the distortion or the image size is missing (empty),
    //     throw std::runtime_error: a half-read calibration is worse than none
    (void)path;
    throw std::logic_error("CameraIntrinsics::load: not implemented yet");
}

CharucoCalibrator::CharucoCalibrator(const CharucoBoardSpec& spec)
    : board_(makeCharucoBoard(spec)), detector_(board_) {}

bool CharucoCalibrator::addView(const cv::Mat& image) {
    // TODO 2 (lesson 2): detect the board and keep its corners.
    //   1. If image_size_ is already set (not empty) and image.size() differs:
    //      throw std::invalid_argument (one calibration = one camera resolution).
    //   2. detector_.detectBoard(image, charuco_corners, charuco_ids)
    //      with std::vector<cv::Point2f> corners and std::vector<int> ids.
    //   3. Fewer than kMinCornersPerView corners: return false (and change nothing).
    //   4. board_.matchImagePoints(corners, ids, object_points, image_points) pairs every
    //      detected corner with its 3D position on the board (std::vector<cv::Point3f> /
    //      std::vector<cv::Point2f>).
    //   5. Remember the image size, append both vectors to object_points_ / image_points_,
    //      return true.
    (void)image;
    throw std::logic_error("CharucoCalibrator::addView: not implemented yet");
}

CameraIntrinsics CharucoCalibrator::calibrate() const {
    // TODO 3 (lesson 3): run the calibration.
    //   - fewer than kMinViews views: throw std::runtime_error (say how many there are)
    //   - cv::calibrateCamera(object_points_, image_points_, image_size_,
    //                         camera_matrix, dist_coeffs, rvecs, tvecs)
    //     fills the camera matrix and distortion and returns the RMS reprojection error.
    //     rvecs / tvecs (std::vector<cv::Mat>) are the board poses, not needed here.
    throw std::logic_error("CharucoCalibrator::calibrate: not implemented yet");
}

}  // namespace armvision::vision
