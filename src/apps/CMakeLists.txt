include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_app_default(format_converter src/format_converter.cpp)
target_link_libraries(format_converter util 3dvideo ${OpenCV_LIBS})

if (RSSDK_FOUND)
    add_app_default(realsense_grabber_app src/realsense_grabber_app.cpp)
    target_link_libraries(realsense_grabber_app util realsense ${OpenCV_LIBS})
endif()

add_app_default(dataset_visualizer_app src/dataset_visualizer_app.cpp)
target_link_libraries(dataset_visualizer_app 3dvideo ${OpenCV_LIBS})

add_app_default(4d_player_app src/4d_player_app.cpp)
target_link_libraries(4d_player_app 3dvideo tri ${OPENGL_LIBRARIES})

# TODO REMOVE
add_app_default(_test_app src/_test_app.cpp)
target_link_libraries(_test_app 3dvideo ${OpenCV_LIBS})