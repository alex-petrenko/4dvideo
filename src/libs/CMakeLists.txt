add_subdirectory(tri)
add_subdirectory(util)
add_subdirectory(4d)

if (RSSDK_FOUND)
    add_subdirectory(realsense)
endif()