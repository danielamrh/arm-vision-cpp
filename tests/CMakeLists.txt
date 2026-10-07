add_executable(armvision_tests
    test_kinematics.cpp
    test_protocol.cpp)
target_link_libraries(armvision_tests PRIVATE armvision::core GTest::gtest_main)
armvision_set_warnings(armvision_tests)

include(GoogleTest)
gtest_discover_tests(armvision_tests)
