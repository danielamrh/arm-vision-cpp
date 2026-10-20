include(FetchContent)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)  # extracted files get the extraction time as timestamp
endif()

# ── Eigen (header-only linear algebra) ────────────────────────
# Use an installed copy if there is one, otherwise download the sources.
# Only the headers are needed, so Eigen's own CMake project is not run.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "Eigen3 not found -> fetching 3.4.0")
    FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# ── GoogleTest ────────────────────────────────────────────────
# Use an installed copy (e.g. Debian's libgtest-dev on the Pi), otherwise download it.
if(ARMVISION_BUILD_TESTS)
    find_package(GTest CONFIG QUIET)
    if(NOT TARGET GTest::gtest_main)
        message(STATUS "GTest not found -> fetching 1.15.2")
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # match MSVC's default /MD runtime
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
        FetchContent_MakeAvailable(googletest)
    endif()
endif()

# ── OpenCV (optional) ─────────────────────────────────────────
# Not fetched: it is large. Linux: apt install libopencv-dev.
# Windows: install the official release and pass -DOpenCV_DIR=<...>/build.
set(ARMVISION_HAVE_OPENCV OFF)
if(ARMVISION_BUILD_VISION)
    find_package(OpenCV 4.7 QUIET
        COMPONENTS core imgproc imgcodecs highgui videoio objdetect calib3d)
    if(OpenCV_FOUND)
        message(STATUS "OpenCV ${OpenCV_VERSION} found -> building vision targets")
        set(ARMVISION_HAVE_OPENCV ON)
    else()
        message(STATUS "OpenCV >= 4.7 not found -> skipping vision targets "
                       "(set OpenCV_DIR to enable them)")
    endif()
endif()
