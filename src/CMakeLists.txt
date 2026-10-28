add_library(armvision_core
    arm_config.cpp
    kinematics.cpp
    protocol.cpp
    robot_driver.cpp)
add_library(armvision::core ALIAS armvision_core)

# Exactly one SerialPort implementation per platform.
if(WIN32)
    target_sources(armvision_core PRIVATE serial_port_win.cpp)
else()
    target_sources(armvision_core PRIVATE serial_port_posix.cpp)
endif()

target_include_directories(armvision_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(armvision_core PUBLIC Eigen3::Eigen)
target_compile_features(armvision_core PUBLIC cxx_std_20)
if(WIN32)
    target_compile_definitions(armvision_core PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
armvision_set_warnings(armvision_core)

if(ARMVISION_HAVE_OPENCV)
    add_library(armvision_vision
        vision/calibration.cpp
        vision/marker_detector.cpp)
    add_library(armvision::vision ALIAS armvision_vision)

    target_include_directories(armvision_vision PUBLIC ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(armvision_vision PUBLIC armvision_core ${OpenCV_LIBS})
    armvision_set_warnings(armvision_vision)
endif()
