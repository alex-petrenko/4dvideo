include(cmake/util.cmake)
common_settings()

add_subdirectory(3rdparty)
add_subdirectory(apps)
add_subdirectory(libs)
add_subdirectory(test)
