#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

namespace armvision::vision {

struct MarkerDetection {
    int id;
    std::array<cv::Point2f, 4> corners;  // clockwise, starting top-left (OpenCV order)
};

/// Thin wrapper around cv::aruco::ArucoDetector (OpenCV >= 4.7).
class MarkerDetector {
public:
    explicit MarkerDetector(
        cv::aruco::PredefinedDictionaryType dictionary = cv::aruco::DICT_4X4_50);

    [[nodiscard]] std::vector<MarkerDetection> detect(const cv::Mat& image) const;

    static void draw(cv::Mat& image, const std::vector<MarkerDetection>& detections);

private:
    cv::aruco::ArucoDetector detector_;
};

}  // namespace armvision::vision
