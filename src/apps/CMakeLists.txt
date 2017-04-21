include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_app_default(3d_video_player src/3d_video_player.cpp)
target_link_libraries(3d_video_player tri ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES} ${OpenCV_LIBS})

add_app_default(format_converter src/format_converter.cpp)
target_link_libraries(format_converter util 3dvideo ${OpenCV_LIBS})

add_app_default(realsense_grabber_app src/realsense_grabber_app.cpp)
target_link_libraries(realsense_grabber_app util realsense ${OpenCV_LIBS})

add_app_default(dataset_visualizer_app src/dataset_visualizer_app.cpp)
target_link_libraries(dataset_visualizer_app 3dvideo ${OpenCV_LIBS})