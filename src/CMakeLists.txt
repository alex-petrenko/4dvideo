if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

    add_definitions(-D_SCL_SECURE_NO_WARNINGS)  # VS annoying warnings
    add_definitions(-D_USE_MATH_DEFINES)  # to enable stuff like M_PI
endif()

add_subdirectory(3rdparty)
add_subdirectory(apps)
add_subdirectory(libs)
add_subdirectory(test)