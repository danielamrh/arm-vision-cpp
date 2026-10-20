add_executable(armvision_tests
    test_kinematics.cpp
    test_protocol.cpp)
target_link_libraries(armvision_tests PRIVATE armvision::core GTest::gtest_main)

if(UNIX)
    target_sources(armvision_tests PRIVATE test_serial_posix.cpp)
endif()

if(ARMVISION_HAVE_OPENCV)
    target_sources(armvision_tests PRIVATE test_vision.cpp)
    target_link_libraries(armvision_tests PRIVATE armvision::vision)
    if(WIN32)
        add_custom_command(TARGET armvision_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:armvision_tests> $<TARGET_FILE_DIR:armvision_tests>
            COMMAND_EXPAND_LISTS)
    endif()
endif()
armvision_set_warnings(armvision_tests)

include(GoogleTest)
gtest_discover_tests(armvision_tests)
