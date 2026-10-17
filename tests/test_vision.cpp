#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "armvision/vision/marker_detector.hpp"

namespace armvision::vision {
namespace {

/// White canvas with ArUco marker `id` (DICT_4X4_50) pasted at `topLeft`.
cv::Mat syntheticScene(int id, cv::Point topLeft, int size) {
    cv::Mat marker;
    cv::aruco::generateImageMarker(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), id, size, marker, 1);
    cv::Mat scene(480, 640, CV_8UC1, cv::Scalar(255));
    marker.copyTo(scene(cv::Rect(topLeft, cv::Size(size, size))));
    cv::Mat bgr;
    cv::cvtColor(scene, bgr, cv::COLOR_GRAY2BGR);  // like a camera frame
    return bgr;
}

TEST(MarkerDetector, FindsSyntheticMarkerWithCorrectIdAndCorners) {
    const cv::Point tl{200, 100};
    const int size = 160;
    const MarkerDetector detector;

    const auto detections = detector.detect(syntheticScene(7, tl, size));

    ASSERT_EQ(detections.size(), 1u);
    EXPECT_EQ(detections[0].id, 7);
    // First corner is the marker's top-left, then clockwise.
    EXPECT_NEAR(detections[0].corners[0].x, tl.x, 2.0);
    EXPECT_NEAR(detections[0].corners[0].y, tl.y, 2.0);
    EXPECT_NEAR(detections[0].corners[2].x, tl.x + size, 2.0);
    EXPECT_NEAR(detections[0].corners[2].y, tl.y + size, 2.0);
}

TEST(MarkerDetector, FindsNothingInEmptyImage) {
    const MarkerDetector detector;
    const cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    EXPECT_TRUE(detector.detect(blank).empty());
}

}  // namespace
}  // namespace armvision::vision
