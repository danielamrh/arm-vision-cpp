// make_charuco_board — writes the calibration board as a print-ready PNG.
//
//   make_charuco_board [out=charuco_board.png]
//
// The image is made for 300 dpi: print it at 100 % ("actual size", no "fit to page") on A4
// landscape, then measure one chessboard square with a ruler and pass it to calibrate_camera
// via --square if it is not exactly 30 mm.

#include <cstdlib>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "armvision/vision/calibration.hpp"

int main(int argc, char** argv) {
    const std::string out = argc > 1 ? argv[1] : "charuco_board.png";

    const armvision::vision::CharucoBoardSpec spec;  // 7 x 5 squares of 30 mm
    constexpr double kDotsPerMetre = 300.0 / 0.0254;  // 300 dpi
    const int width = cvRound(spec.squares_x * spec.square_length * kDotsPerMetre);
    const int height = cvRound(spec.squares_y * spec.square_length * kDotsPerMetre);
    const int margin = cvRound(0.010 * kDotsPerMetre);  // 10 mm white border

    cv::Mat board;
    armvision::vision::makeCharucoBoard(spec).generateImage(cv::Size(width, height), board, 0, 1);
    cv::copyMakeBorder(board, board, margin, margin, margin, margin, cv::BORDER_CONSTANT,
                       cv::Scalar(255));

    if (!cv::imwrite(out, board)) {
        std::cerr << "cannot write " << out << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "wrote " << out << " (" << board.cols << " x " << board.rows
              << " px = 300 dpi, print at 100 %)\n";
    return EXIT_SUCCESS;
}
