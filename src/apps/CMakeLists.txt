include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_app_default(format_converter_app src/format_converter_app.cpp)
target_link_libraries(format_converter_app util 4d ${OpenCV_LIBS})

if (RSSDK_FOUND)
    add_app_default(realsense_grabber_app src/realsense_grabber_app.cpp)
    target_link_libraries(realsense_grabber_app util realsense ${OpenCV_LIBS})
endif()

add_app_default(dataset_visualizer_app src/dataset_visualizer_app.cpp)
target_link_libraries(dataset_visualizer_app 4d ${OpenCV_LIBS})

add_app_default(4d_player_app src/4d_player_app.cpp)
target_link_libraries(4d_player_app 4d tri ${OPENGL_LIBRARIES})

add_app_default(animation_writer_app src/animation_writer_app.cpp)
target_link_libraries(animation_writer_app 4d tri ${OPENGL_LIBRARIES})

add_app_default(triangulation_visualizer_app src/triangulation_visualizer_app.cpp)
target_link_libraries(triangulation_visualizer_app tri)

add_app_default(sandbox_app src/sandbox_app.cpp)
target_link_libraries(sandbox_app 4d ${OpenCV_LIBS})