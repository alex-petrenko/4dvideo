include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_app_default(3d_video_player)
target_link_libraries(3d_video_player tri ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} ${OpenCV_LIBS})
