project(lib3dvideo VERSION 0.1 LANGUAGES CXX)

include_directories(${GLFW3_INCLUDE_PATH} ${GLEW_INCLUDE_DIRS} ${GLM_INCLUDES})

add_library_default(3dvideo)
target_link_libraries(3dvideo tri util)
