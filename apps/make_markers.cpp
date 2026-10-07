// make_markers — writes printable ArUco markers (DICT_4X4_50) to markers/.
//
//   make_markers [count=4] [size_px=400]

#include <cstdlib>
#include <filesystem>
#include <format>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

int main(int argc, char** argv) {
    const int count = argc > 1 ? std::atoi(argv[1]) : 4;
    const int size = argc > 2 ? std::atoi(argv[2]) : 400;

    const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::filesystem::create_directories("markers");

    for (int id = 0; id < count; ++id) {
        cv::Mat img;
        cv::aruco::generateImageMarker(dict, id, size, img, 1);
        // White quiet zone around the marker makes detection much more reliable.
        cv::copyMakeBorder(img, img, size / 8, size / 8, size / 8, size / 8, cv::BORDER_CONSTANT,
                           cv::Scalar(255));
        const auto path = std::format("markers/aruco_4x4_{:02}.png", id);
        cv::imwrite(path, img);
        std::cout << "wrote " << path << '\n';
    }
    return EXIT_SUCCESS;
}
