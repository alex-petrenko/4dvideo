project(libutil VERSION 0.1 LANGUAGES CXX)

include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_library_default(util)
target_link_libraries(util ${GLFW_LIBRARIES} ${GLEW_LIBRARIES})
