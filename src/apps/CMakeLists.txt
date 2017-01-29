include_directories(SYSTEM ${GLFW3_INCLUDE_PATH})

add_app_default(3d_video_player)
target_link_libraries(3d_video_player tri ${GLFW_LIBRARIES})
