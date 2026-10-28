// calibrate_camera — camera calibration with a printed ChArUco board (see make_charuco_board).
//
//   calibrate_camera live [camera_index=0] [options]      webcam with live preview
//   calibrate_camera images <folder> [options]            photos, e.g. taken on the Pi
//
// options:
//   --out <file>        result file (default camera.yaml)
//   --square <metres>   measured square length of the print (default 0.030)
//   --marker <metres>   measured marker length of the print (default 0.022)
//
// live keys: space = use this frame, c = calibrate + save, u = toggle undistorted view, q = quit

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "armvision/vision/calibration.hpp"

namespace {

using armvision::vision::CameraIntrinsics;
using armvision::vision::CharucoBoardSpec;
using armvision::vision::CharucoCalibrator;

int usage() {
    std::cerr << "usage:\n"
                 "  calibrate_camera live [camera_index] [--out f] [--square m] [--marker m]\n"
                 "  calibrate_camera images <folder>     [--out f] [--square m] [--marker m]\n";
    return EXIT_FAILURE;
}

void report(const CameraIntrinsics& c, const std::string& path) {
    const auto K = cv::Matx33d(c.camera_matrix);
    std::cout << std::format(
        "image {}x{}  fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}  RMS reprojection error {:.3f} px\n",
        c.image_size.width, c.image_size.height, K(0, 0), K(1, 1), K(0, 2), K(1, 2),
        c.rms_reprojection_error);
    std::cout << "distortion (k1 k2 p1 p2 k3): " << c.dist_coeffs << '\n';
    std::cout << "saved to " << path << '\n';
}

int runImages(const std::filesystem::path& folder, const CharucoBoardSpec& spec,
              const std::string& out) {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
        auto ext = entry.path().extension().string();
        std::ranges::transform(ext, ext.begin(), [](unsigned char ch) { return std::tolower(ch); });
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") files.push_back(entry.path());
    }
    std::ranges::sort(files);

    CharucoCalibrator calibrator(spec);
    for (const auto& file : files) {
        const cv::Mat image = cv::imread(file.string());
        const bool used = !image.empty() && calibrator.addView(image);
        std::cout << (used ? "  used    " : "  skipped ") << file.filename().string() << '\n';
    }
    const CameraIntrinsics result = calibrator.calibrate();
    result.save(out);
    report(result, out);
    return EXIT_SUCCESS;
}

int runLive(int camera_index, const CharucoBoardSpec& spec, const std::string& out) {
    cv::VideoCapture cap(camera_index);
    if (!cap.isOpened()) {
        std::cerr << "cannot open camera " << camera_index << '\n';
        return EXIT_FAILURE;
    }

    CharucoCalibrator calibrator(spec);
    const cv::aruco::CharucoDetector preview(armvision::vision::makeCharucoBoard(spec));
    std::optional<CameraIntrinsics> calibration;
    bool show_undistorted = false;

    cv::Mat frame, display;
    while (cap.read(frame)) {
        // Live feedback: which board corners are visible right now?
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        preview.detectBoard(frame, corners, ids);

        if (show_undistorted && calibration) {
            cv::undistort(frame, display, calibration->camera_matrix, calibration->dist_coeffs);
        } else {
            display = frame.clone();
            if (!ids.empty()) cv::aruco::drawDetectedCornersCharuco(display, corners, ids);
        }
        cv::putText(display,
                    std::format("views {}  corners {}  [space] add  [c] calibrate  [u] undistort",
                                calibrator.viewCount(), ids.size()),
                    {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 0}, 2);
        cv::imshow("calibrate_camera", display);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        if (key == ' ') {
            std::cout << (calibrator.addView(frame) ? "view added" : "board not found well enough")
                      << " (" << calibrator.viewCount() << " views)\n";
        } else if (key == 'c') {
            try {
                calibration = calibrator.calibrate();
                calibration->save(out);
                report(*calibration, out);
            } catch (const std::exception& e) {
                std::cout << e.what() << '\n';
            }
        } else if (key == 'u') {
            show_undistorted = !show_undistorted;
        }
    }
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) try {
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    if (args.empty()) return usage();

    CharucoBoardSpec spec;
    std::string out = "camera.yaml";
    std::vector<std::string_view> positional;
    for (std::size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--out" && i + 1 < args.size()) {
            out = args[++i];
        } else if (args[i] == "--square" && i + 1 < args.size()) {
            spec.square_length = std::stof(std::string{args[++i]});
        } else if (args[i] == "--marker" && i + 1 < args.size()) {
            spec.marker_length = std::stof(std::string{args[++i]});
        } else {
            positional.push_back(args[i]);
        }
    }

    if (positional.empty()) return usage();
    if (positional[0] == "images" && positional.size() == 2) {
        return runImages(std::filesystem::path(positional[1]), spec, out);
    }
    if (positional[0] == "live" && positional.size() <= 2) {
        const int index = positional.size() == 2 ? std::stoi(std::string{positional[1]}) : 0;
        return runLive(index, spec, out);
    }
    return usage();
} catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
}
