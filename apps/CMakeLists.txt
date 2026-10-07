add_executable(arm_cli arm_cli.cpp)
target_link_libraries(arm_cli PRIVATE armvision::core)
armvision_set_warnings(arm_cli)

if(ARMVISION_HAVE_OPENCV)
    foreach(app camera_view make_markers)
        add_executable(${app} ${app}.cpp)
        target_link_libraries(${app} PRIVATE armvision::vision)
        armvision_set_warnings(${app})

        # Copy the OpenCV DLLs next to the executable so it starts without PATH changes.
        if(WIN32)
            add_custom_command(TARGET ${app} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_RUNTIME_DLLS:${app}> $<TARGET_FILE_DIR:${app}>
                COMMAND_EXPAND_LISTS)
        endif()
    endforeach()
endif()
