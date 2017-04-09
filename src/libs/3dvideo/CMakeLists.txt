project(lib3dvideo VERSION 0.1 LANGUAGES CXX)

add_library_default(3dvideo)
target_link_libraries(3dvideo util ${OpenCV_LIBS})
