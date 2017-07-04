project(librealsense VERSION 0.1 LANGUAGES CXX)

include_directories(${RSSDK_INCLUDES})
add_library_default(realsense)
target_link_libraries(realsense 4d util ${RSSDK_LIBS} ${OpenCV_LIBS})
