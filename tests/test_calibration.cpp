// Tests for the ChArUco camera calibration. They need no camera: the board is rendered into
// synthetic images with a known, ideal pinhole camera, so we know exactly what the calibration
// has to find.

#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "armvision/vision/calibration.hpp"

namespace armvision::vision {
namespace {

const cv::Size kImageSize{640, 480};
const cv::Matx33d kTrueK{600.0, 0.0, 320.0,  //
                         0.0, 600.0, 240.0,  //
                         0.0, 0.0, 1.0};

/// Renders the board as seen by the ideal camera kTrueK. The board centre sits `distance` metres
/// in front of the camera, tilted by the given angles (degrees) around its own centre.
cv::Mat renderView(const CharucoBoardSpec& spec, double ax, double ay, double az, double distance) {
    constexpr double kPxPerMetre = 5000.0;  // resolution of the "printed" board
    const double width = spec.squares_x * spec.square_length;
    const double height = spec.squares_y * spec.square_length;

    cv::Mat board_image;
    makeCharucoBoard(spec).generateImage(
        cv::Size(cvRound(width * kPxPerMetre), cvRound(height * kPxPerMetre)), board_image, 0, 1);

    // Board coordinates (metres, z = 0) -> camera: x_cam = R (X - c) + (0, 0, distance).
    const double deg = CV_PI / 180.0;
    cv::Matx33d R;
    cv::Rodrigues(cv::Vec3d(ax * deg, ay * deg, az * deg), R);
    const cv::Vec3d centre(width / 2, height / 2, 0.0);
    const cv::Vec3d t = cv::Vec3d(0, 0, distance) - R * centre;

    // For points on the plane z = 0 the projection is a homography: K [r1 r2 t].
    const cv::Matx33d Rt(R(0, 0), R(0, 1), t[0], R(1, 0), R(1, 1), t[1], R(2, 0), R(2, 1), t[2]);
    const cv::Matx33d px_to_metre(1.0 / kPxPerMetre, 0, 0, 0, 1.0 / kPxPerMetre, 0, 0, 0, 1);
    const cv::Matx33d H = kTrueK * Rt * px_to_metre;

    cv::Mat view;
    cv::warpPerspective(board_image, view, cv::Mat(H), kImageSize, cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar(255));
    cv::cvtColor(view, view, cv::COLOR_GRAY2BGR);  // like a camera frame
    return view;
}

/// A set of well spread poses: strong tilts in all directions, some rotation, different
/// distances. Views straight from the front alone cannot pin down the principal point.
std::vector<cv::Mat> syntheticViews(const CharucoBoardSpec& spec) {
    const double poses[][4] = {
        {0, 0, 0, 0.35},      {35, 0, 0, 0.36},    {-35, 0, 0, 0.36},   {0, 35, 0, 0.37},
        {0, -35, 0, 0.37},    {25, 25, 10, 0.4},   {-25, 25, -10, 0.38}, {25, -25, 5, 0.39},
        {-25, -25, 0, 0.42},  {30, 10, -15, 0.4},  {-10, 30, 20, 0.38}, {10, -30, -20, 0.38},
    };
    std::vector<cv::Mat> views;
    for (const auto& p : poses) views.push_back(renderView(spec, p[0], p[1], p[2], p[3]));
    return views;
}

// ── Lesson 1 ─────────────────────────────────────────────────

TEST(Calibration, BoardHasOneCornerPerInnerChessboardCrossing) {
    const CharucoBoardSpec spec;  // 7 x 5 squares
    const auto board = makeCharucoBoard(spec);
    EXPECT_EQ(board.getChessboardSize(), cv::Size(7, 5));
    EXPECT_EQ(board.getChessboardCorners().size(), 6u * 4u);
    EXPECT_FLOAT_EQ(board.getSquareLength(), spec.square_length);
}

// ── Lesson 2 ─────────────────────────────────────────────────

TEST(Calibration, AddViewAcceptsAnImageOfTheBoard) {
    const CharucoBoardSpec spec;
    CharucoCalibrator calibrator(spec);
    EXPECT_TRUE(calibrator.addView(renderView(spec, 10, -10, 0, 0.35)));
    EXPECT_EQ(calibrator.viewCount(), 1u);
}

TEST(Calibration, AddViewRejectsAnImageWithoutBoard) {
    CharucoCalibrator calibrator;
    const cv::Mat blank(kImageSize, CV_8UC3, cv::Scalar(255, 255, 255));
    EXPECT_FALSE(calibrator.addView(blank));
    EXPECT_EQ(calibrator.viewCount(), 0u);
}

TEST(Calibration, AddViewRejectsImagesOfDifferentSize) {
    const CharucoBoardSpec spec;
    CharucoCalibrator calibrator(spec);
    ASSERT_TRUE(calibrator.addView(renderView(spec, 0, 0, 0, 0.35)));
    cv::Mat smaller;
    cv::resize(renderView(spec, 0, 0, 0, 0.35), smaller, cv::Size(320, 240));
    EXPECT_THROW(calibrator.addView(smaller), std::invalid_argument);
}

// ── Lesson 3 ─────────────────────────────────────────────────

TEST(Calibration, CalibrateNeedsEnoughViews) {
    const CharucoBoardSpec spec;
    CharucoCalibrator calibrator(spec);
    ASSERT_TRUE(calibrator.addView(renderView(spec, 0, 0, 0, 0.35)));
    EXPECT_THROW((void)calibrator.calibrate(), std::runtime_error);
}

TEST(Calibration, RecoversTheKnownCamera) {
    const CharucoBoardSpec spec;
    CharucoCalibrator calibrator(spec);
    for (const auto& view : syntheticViews(spec)) ASSERT_TRUE(calibrator.addView(view));

    const CameraIntrinsics result = calibrator.calibrate();
    ASSERT_EQ(result.camera_matrix.size(), cv::Size(3, 3));
    const auto K = cv::Matx33d(result.camera_matrix);
    // Even with perfect images the principal point is the least certain value (here ~3 px off),
    // because it trades off against the tilt of the views and the distortion coefficients.
    EXPECT_NEAR(K(0, 0), kTrueK(0, 0), 6.0);  // fx within 1 %
    EXPECT_NEAR(K(1, 1), kTrueK(1, 1), 6.0);  // fy
    EXPECT_NEAR(K(0, 2), kTrueK(0, 2), 6.0);  // cx within 1 % of the image width
    EXPECT_NEAR(K(1, 2), kTrueK(1, 2), 6.0);  // cy
    EXPECT_LT(result.rms_reprojection_error, 0.5);
    EXPECT_EQ(result.image_size, kImageSize);
}

// ── Lesson 4 ─────────────────────────────────────────────────

TEST(Calibration, SaveAndLoadRoundTrip) {
    CameraIntrinsics original;
    original.image_size = kImageSize;
    original.camera_matrix = cv::Mat(kTrueK);
    original.dist_coeffs = cv::Mat(cv::Matx<double, 1, 5>(0.1, -0.2, 0.001, 0.002, 0.05));
    original.rms_reprojection_error = 0.27;

    const auto path = std::filesystem::temp_directory_path() / "armvision_test_camera.yaml";
    original.save(path.string());
    const CameraIntrinsics loaded = CameraIntrinsics::load(path.string());
    std::filesystem::remove(path);

    EXPECT_EQ(loaded.image_size, original.image_size);
    EXPECT_EQ(cv::norm(loaded.camera_matrix, original.camera_matrix), 0.0);
    EXPECT_EQ(cv::norm(loaded.dist_coeffs, original.dist_coeffs), 0.0);
    EXPECT_DOUBLE_EQ(loaded.rms_reprojection_error, original.rms_reprojection_error);
}

TEST(Calibration, LoadThrowsForMissingFile) {
    EXPECT_THROW((void)CameraIntrinsics::load("does/not/exist.yaml"), std::runtime_error);
}

}  // namespace
}  // namespace armvision::vision
