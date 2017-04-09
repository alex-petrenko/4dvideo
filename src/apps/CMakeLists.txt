include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_app_default(3d_video_player src/3d_video_player.cpp)
target_link_libraries(3d_video_player tri ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} ${OpenCV_LIBS})

add_app_default(format_converter src/format_converter.cpp)
target_link_libraries(format_converter util ${OpenCV_LIBS})

include_directories(${RSSDK_INCLUDES})
add_app_default(realsense_grabber src/realsense_grabber.cpp)
target_link_libraries(realsense_grabber util ${OpenCV_LIBS} ${RSSDK_LIBS})