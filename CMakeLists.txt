cmake_minimum_required(VERSION 3.21)

project(armvision
    VERSION 0.1.0
    DESCRIPTION "Camera-guided control of a 4-DOF hobby robot arm in modern C++"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ARMVISION_BUILD_TESTS "Build the unit tests" ON)
option(ARMVISION_BUILD_VISION "Build the OpenCV-based targets (needs OpenCV >= 4.7)" ON)
option(ARMVISION_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

include(cmake/CompilerWarnings.cmake)
include(cmake/Dependencies.cmake)

add_subdirectory(src)
add_subdirectory(apps)

if(ARMVISION_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
