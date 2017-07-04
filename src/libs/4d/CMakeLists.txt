project(lib4d VERSION 0.1 LANGUAGES CXX)

include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_library_default(4d)
target_link_libraries(4d tri util)
