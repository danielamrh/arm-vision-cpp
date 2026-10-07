#include "armvision/vision/marker_detector.hpp"

#include <algorithm>

namespace armvision::vision {

MarkerDetector::MarkerDetector(cv::aruco::PredefinedDictionaryType dictionary)
    : detector_(cv::aruco::getPredefinedDictionary(dictionary),
                cv::aruco::DetectorParameters{}) {}

std::vector<MarkerDetection> MarkerDetector::detect(const cv::Mat& image) const {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_.detectMarkers(image, corners, ids);

    std::vector<MarkerDetection> out;
    out.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        MarkerDetection d{ids[i], {}};
        std::copy_n(corners[i].begin(), 4, d.corners.begin());
        out.push_back(d);
    }
    return out;
}

void MarkerDetector::draw(cv::Mat& image, const std::vector<MarkerDetection>& detections) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    for (const auto& d : detections) {
        ids.push_back(d.id);
        corners.emplace_back(d.corners.begin(), d.corners.end());
    }
    cv::aruco::drawDetectedMarkers(image, corners, ids);
}

}  // namespace armvision::vision
