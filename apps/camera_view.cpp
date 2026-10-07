// camera_view — live webcam view with ArUco detection.
//
//   camera_view [camera_index=0]
//   keys: s = save frame to captures/, q / Esc = quit

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "armvision/vision/marker_detector.hpp"

int main(int argc, char** argv) {
    const int index = argc > 1 ? std::atoi(argv[1]) : 0;

    cv::VideoCapture cap(index);
    if (!cap.isOpened()) {
        std::cerr << "cannot open camera " << index << '\n';
        return EXIT_FAILURE;
    }

    const armvision::vision::MarkerDetector detector;
    cv::Mat frame;
    int saved = 0;
    auto last = std::chrono::steady_clock::now();
    double fps = 0.0;

    while (cap.read(frame)) {
        const auto detections = detector.detect(frame);
        armvision::vision::MarkerDetector::draw(frame, detections);

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;
        fps = 0.9 * fps + 0.1 * (dt > 0.0 ? 1.0 / dt : 0.0);  // smoothed
        cv::putText(frame, std::format("{:.1f} fps, {} markers", fps, detections.size()),
                    {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

        cv::imshow("camera_view", frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        if (key == 's') {
            std::filesystem::create_directories("captures");
            const auto path = std::format("captures/frame_{:03}.png", saved++);
            cv::imwrite(path, frame);
            std::cout << "saved " << path << '\n';
        }
    }
    return EXIT_SUCCESS;
}
